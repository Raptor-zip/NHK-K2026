"""ニップ隙間の管理精度 — 調整機構の分解能・摩耗・実測手段.

    python scripts/nip_calibration.py [--md]

誤差バジェット（scripts/accuracy_budget.py）で「命中率は初速の再現性で決まり、
厚みフィードフォワードを入れると次はニップ隙間の管理が支配的になる」と分かった。
ここでは **隙間を 1% (0.015mm) で管理できるのか** を機構side から詰める。

見るもの
  1. 調整機構の分解能: 偏心カム / ねじ押し / テーパくさび の比較
  2. 摩耗と圧縮永久ひずみによる隙間のドリフト
  3. 射出時のローラー回転数降下から初速を実測できるか（既存エンコーダのみ）
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import tr_params as P  # noqa: E402

GAP_MM = P.NIP_GAP           # 1.5
TARGET_REL = 0.01            # 隙間の管理目標（1% = 0.015mm）
TARGET_MM = GAP_MM * TARGET_REL

# --- ローラー諸元 ---------------------------------------------------------
J_AXIS = 6.9e-4              # kg·m² 1軸（ローラー5個＋中空軸）
R_ROLL = P.ROLLER_DIA / 2 * 1e-3
V_MUZZLE = 8.37              # m/s 設計初速（旗・仰角67°）
M_RAG = P.RAG_MASS
RPM_RESOLUTION = 1.0         # C620 が返す回転数の分解能 [rpm]


def adjusters():
    """(方式, 1回転あたりの隙間変化, 実用的な操作分解能, 得られる隙間分解能, 備考)"""
    out = []
    # 偏心カム: 偏心量 e、目盛り n 分割 → 最悪 e·(2π/n)
    for e, n in ((1.0, 30), (0.5, 60)):
        res = e * (2 * math.pi / n)
        out.append((f"偏心カム e={e}mm・{n}分割", 2 * e, f"1/{n} 回転", res,
                    "現行案。手回しでは目盛りが読めても再現しにくい"))
    # ねじ直押し: ピッチ p、1/k 回転
    for p_, k in ((0.7, 12), (0.5, 20)):
        out.append((f"ねじ直押し M4×{p_}mm・1/{k}回転", p_, f"1/{k} 回転", p_ / k,
                    "ロックナット必須。緩むと隙間が変わる"))
    # 差動ねじ: p1 - p2
    out.append(("差動ねじ M5×0.8 と M6×1.0", 0.2, "1/20 回転", 0.2 / 20,
                "部品が増えるが自立保持。加工難度は中"))
    # テーパくさび: ねじピッチ p × テーパ比 1:t
    for p_, t in ((0.7, 20), (0.7, 40)):
        out.append((f"テーパくさび 1:{t}（M4×{p_}mm 駆動）", p_ / t, "1/10 回転", p_ / t / 10,
                    "CNCで作れる。自己保持。**推奨**"))
    return out


def wear_drift():
    """摩耗・圧縮永久ひずみによる隙間のドリフト見積もり。"""
    return [
        ("ウレタンの圧縮永久ひずみ（1試合 32射）", 0.005, "初期なじみ。試合前に再調整すれば効かない"),
        ("ウレタンの摩耗（1日 300射）", 0.020, "布は柔らかいので摩耗は小さい。1日で 0.02mm 級"),
        ("温度による寸法変化（ΔT=15℃）", 0.008, "ウレタンの線膨張 α≈150e-6、厚み3mm ぶん"),
        ("軸のたわみ（射出時のニップ反力）", 0.012, "φ12中空軸・支持間620mm・反力40N の中央たわみ"),
    ]


def speed_from_rpm_drop():
    """射出時のローラー回転数降下から初速を推定する精度。"""
    omega = V_MUZZLE / R_ROLL
    e_rag = 0.5 * M_RAG * V_MUZZLE ** 2          # 雑巾に渡るエネルギー [J]
    j_tot = 2 * J_AXIS                            # 上下2軸
    d_omega = e_rag / (j_tot * omega)             # 角速度の降下 [rad/s]
    res_rad = RPM_RESOLUTION * 2 * math.pi / 60   # エンコーダ分解能 [rad/s]
    rel_domega = res_rad / d_omega                # Δω の相対分解能
    # E ∝ v² なので v の相対誤差は 1/2
    return omega, d_omega, d_omega / omega, rel_domega, rel_domega / 2


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()
    md = args.md

    rows = adjusters()
    if md:
        print(f"## 1. 調整機構の分解能（目標: 隙間 {GAP_MM}mm を ±{TARGET_MM * 1000:.0f}µm で管理）\n")
        print("| 方式 | 1回転あたり | 操作分解能 | 得られる隙間分解能 | 目標比 | 備考 |")
        print("|---|---|---|---|---|---|")
        for name, per_rev, op, res, note in rows:
            print(f"| {name} | {per_rev:.3f} mm | {op} | **{res * 1000:.0f} µm** | "
                  f"{'✅' if res <= TARGET_MM else '❌ ' + f'{res / TARGET_MM:.0f}倍粗い'} | {note} |")
    else:
        print(f"目標 隙間分解能 {TARGET_MM * 1000:.0f} µm")
        for name, per_rev, op, res, note in rows:
            print(f"  {name:34s} {res * 1000:6.1f} µm  "
                  f"{'OK' if res <= TARGET_MM else f'NG({res / TARGET_MM:.0f}x)'}")

    w = wear_drift()
    total = math.sqrt(sum(x[1] ** 2 for x in w))
    if md:
        print(f"\n## 2. 隙間のドリフト（管理しないと効いてくる分）\n")
        print("| 要因 | 隙間変化 [mm] | 初速換算 | 備考 |")
        print("|---|---|---|---|")
        for name, d, note in w:
            print(f"| {name} | {d:.3f} | {d / GAP_MM * 0.45 * 100:.2f}% | {note} |")
        print(f"| **二乗和（RSS）** | **{total:.3f}** | "
              f"**{total / GAP_MM * 0.45 * 100:.2f}%** | 目標 1% に対して |")
    else:
        print(f"\n  ドリフト RSS {total:.3f} mm = 初速 {total / GAP_MM * 0.45 * 100:.2f}%")

    omega, d_omega, rel_drop, rel_res, rel_v = speed_from_rpm_drop()
    if md:
        print(f"\n## 3. 射出ごとの初速の実測（既存エンコーダだけで可能か）\n")
        print("射出でローラーの運動エネルギーが雑巾へ移る。その回転数降下を測れば初速が逆算できる。\n")
        print("| 項目 | 値 |")
        print("|---|---|")
        print(f"| 定常角速度 | {omega:.0f} rad/s（{omega * 60 / 2 / math.pi:.0f} rpm） |")
        print(f"| 雑巾へ渡るエネルギー | {0.5 * M_RAG * V_MUZZLE ** 2:.2f} J |")
        print(f"| 1射あたりの角速度降下 | **{d_omega:.1f} rad/s（{rel_drop * 100:.1f}%）** |")
        print(f"| C620 の回転数分解能 | {RPM_RESOLUTION:.0f} rpm = {RPM_RESOLUTION * 2 * math.pi / 60:.3f} rad/s |")
        print(f"| 降下量の相対分解能 | {rel_res * 100:.1f}% |")
        print(f"| **初速の測定精度（E∝v² なので 1/2）** | **{rel_v * 100:.1f}%** |")
    else:
        print(f"\n  1射の角速度降下 {d_omega:.1f} rad/s ({rel_drop*100:.1f}%) → 初速測定精度 {rel_v*100:.2f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
