"""命中率の誤差バジェット — どの誤差が命中率を決めているかを分解する.

    python scripts/accuracy_budget.py [--md] [--n 20000]

`control/tr_control.py` の ShotSolver（gpusim 較正つき）で狙いを出し、
機構・制御・センサーの誤差を注入して、旗の横棒に「上から被せる」判定を行う。

命中の定義（gpusim の「下降当て」戦略に合わせる）
  * 横棒の位置 x_bar での軌道高さが 3.00〜3.25m（棒の上に出て、布が垂れて被さる範囲）
  * その点で降下中（dz/dx < 0）＝ 頂点を越えている
  * 横方向のずれが ±0.30m 以内（棒の幅600mm）

誤差源（1σ）は実測前の設計値。8月の弾道テーブル本計測で置き換えること。
"""

from __future__ import annotations

import argparse
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "control"))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import tr_control as C  # noqa: E402
import tr_params as P  # noqa: E402

G = 9.81
X_BAR = 3.92          # m 射出点から横棒までの水平距離（gpusim の較正条件）
Z_BAR = 3.00          # m 横棒の高さ（フィールド基準）
RISE_BAR = Z_BAR - C.NIP[2]   # m 射出点からの上昇量（height_at は射出点基準）
CATCH_LO, CATCH_HI = 0.00, 0.25   # m 棒より上 0〜25cm で降下中なら被さる
CATCH_Y = 0.30        # m 横方向の許容（棒の幅600mm）

# --- 誤差源（1σ）--------------------------------------------------------
SOURCES = {
    "初速（ニップ隙間・スリップ）": dict(kind="v_rel", sigma=0.020,
                                        note="隙間1.5±0.2mm の圧縮率差。実測で詰める"),
    "初速（ローラー回転数制御）": dict(kind="v_rel", sigma=0.005,
                                      note="C620 の閉ループ。エンコーダ分解能は十分"),
    "雑巾の個体差（質量・毛羽）": dict(kind="v_rel", sigma=0.015,
                                      note="48g±3g。競技委員会支給で選別不可"),
    # ⚠ この 0.15° は当てずっぽうではなく、**図に描いてある歯すきま**から出る。
    #   片フランクの軸方向すきま PITCH_WORM_BL=0.12mm、全遊びはその 2 倍。
    #   ウォームの軸方向はホイールの接線方向なので、
    #     Δθ = 0.24mm / 30mm(基準円半径) = 0.0080rad = 0.458°
    #   これが一様に分布するとみて 1σ = 0.458/√12 = 0.13°。
    #   バックラッシを詰めるとここが下がるが、詰めすぎると噛み込む
    #   （tr_params の PITCH_WORM_BL の注記）。
    "仰角（ウォーム40:1 バックラッシュ）": dict(
        kind="pitch_deg", sigma=0.15,
        note="全遊び0.458°(片フランク0.12mm×2/R30)の一様分布 1σ=0.13。逆駆動なし"),
    "ヨー（ベルト3:1 バックラッシュ）": dict(kind="yaw_deg", sigma=0.20, note="—"),
    "射出点の変位（骨格のたわみ）": dict(kind="nip_m", sigma=0.0003,
                                        note="静解析の荷重ケース間差 0.73mm の 1σ 相当"),
    "自己位置推定（LiDAR + ICP）": dict(kind="range_m", sigma=0.010, note="目標 ±30mm の 1σ"),
    "布のニップ投入姿勢": dict(kind="pitch_deg", sigma=0.60,
                              note="gpusim の meanMiss 0.044m 相当。**支配的**"),
}


def height_at(x: float, v: float, pitch: float) -> tuple[float, float]:
    """水平距離 x での高さと勾配（真空弾道）。"""
    c = math.cos(pitch)
    z = x * math.tan(pitch) - G * x * x / (2 * v * v * c * c)
    dz = math.tan(pitch) - G * x / (v * v * c * c)
    return z, dz


def hit(v: float, pitch: float, yaw: float, x_bar: float) -> bool:
    z, dz = height_at(x_bar, v, pitch)
    dy = x_bar * math.tan(yaw)
    return (RISE_BAR + CATCH_LO <= z <= RISE_BAR + CATCH_HI) and dz < 0 and abs(dy) <= CATCH_Y


def nominal():
    sol = C.ShotSolver()
    tgt = (sol.nip[0] + X_BAR, 0.0, Z_BAR)
    yaw, pitch, v, err = sol.solve(tgt, prefer_pitch=math.radians(67.0))
    if err:
        raise SystemExit(f"狙い解が出ない: {err}")
    # 以降の軌道計算は真空式なので、**等価な真空初速**に直して扱う。
    # （指令初速 = 真空初速 × 布の揚力補正。誤差は相対量なのでどちらの空間でも同じ）
    v /= sol._pitch_calib(pitch, X_BAR)
    # 「下降当て」なので、棒の位置で 12cm 上に出るよう初速を微増させる
    for _ in range(60):
        z, _dz = height_at(X_BAR, v, pitch)
        if abs(z - (RISE_BAR + 0.12)) < 0.0005:
            break
        v += 0.002 if z < RISE_BAR + 0.12 else -0.002
    return v, pitch, yaw


def run(n: int, only: str | None = None, scale: float = 1.0):
    v0, p0, y0 = nominal()
    rng = random.Random(20260729)
    ok = 0
    for _ in range(n):
        v, pitch, yaw, xb = v0, p0, y0, X_BAR
        for name, spec in SOURCES.items():
            if only is not None and name != only:
                continue
            s = spec["sigma"] * scale
            g = rng.gauss(0.0, 1.0)
            if spec["kind"] == "v_rel":
                v *= 1.0 + s * g
            elif spec["kind"] == "pitch_deg":
                pitch += math.radians(s * g)
            elif spec["kind"] == "yaw_deg":
                yaw += math.radians(s * g)
            elif spec["kind"] == "nip_m":
                xb += s * g
            elif spec["kind"] == "range_m":
                xb += s * g
        if hit(v, pitch, yaw, xb):
            ok += 1
    return ok / n, v0, p0


# --- 厚み計測によるフィードフォワード補正 -------------------------------
# 1枚ごとの雑巾の厚みを測れば、ニップ圧縮率 → スリップ率 → 初速のずれが
# 予測できる。ニップ到達まで 0.33 秒あるので、その間にローラー回転数を補正できる。
NIP_GAP_MM = P.NIP_GAP          # 1.5
RAG_T_MM = P.RAG_T              # 3.0
RAG_T_SIGMA_MM = 0.20           # 厚みの個体差 1σ（実測で確定）
SLIP_COEF = 0.45                # 圧縮率の変化 → 初速の変化 の比（実測で確定）


def feedforward_residual(sensor_res_mm: float) -> float:
    """厚みセンサーの分解能から、補正後に残る初速の 1σ を返す。

    圧縮率 c = (t - gap)/t、 dc/dt = gap/t²
    初速のずれ dv/v = SLIP_COEF · dc
    センサーで測れない分（分解能の 1/√12 が量子化誤差の 1σ）だけが残る。
    """
    dc_dt = NIP_GAP_MM / (RAG_T_MM ** 2)
    unresolved_mm = sensor_res_mm / math.sqrt(12.0)
    return SLIP_COEF * dc_dt * unresolved_mm


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=20000)
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()

    total, v0, p0 = run(args.n)
    rows = []
    for name in SOURCES:
        p_single, _, _ = run(args.n, only=name)
        rows.append((name, SOURCES[name]["sigma"], SOURCES[name]["kind"],
                     p_single, SOURCES[name]["note"]))
    rows.sort(key=lambda r: r[3])   # 単独でも命中率を落とす＝支配的

    if args.md:
        print(f"狙い: 初速 **{v0:.2f} m/s** / 仰角 **{math.degrees(p0):.0f}°**"
              f"（横棒の 12cm 上を降下中に通す）\n")
        print("| 誤差源 | 1σ | 単独での命中率 | 備考 |")
        print("|---|---|---|---|")
        for name, sig, kind, p1, note in rows:
            unit = {"v_rel": "%", "pitch_deg": "°", "yaw_deg": "°",
                    "nip_m": "m", "range_m": "m"}[kind]
            val = f"{sig * 100:.1f}{unit}" if kind == "v_rel" else f"{sig:.3g}{unit}"
            print(f"| {name} | {val} | {p1 * 100:.0f}% | {note} |")
        print(f"| **全誤差を同時に入れた命中率** | | **{total * 100:.0f}%** | |")
        print()
        print("### 初速の再現性を詰めるとどうなるか\n")
        print("| 初速の 1σ | 命中率 |")
        print("|---|---|")
        for sc in (1.0, 0.5, 0.25):
            SOURCES["初速（ニップ隙間・スリップ）"]["sigma"] = 0.020 * sc
            SOURCES["雑巾の個体差（質量・毛羽）"]["sigma"] = 0.015 * sc
            pr, _, _ = run(args.n)
            print(f"| 隙間{2.0 * sc:.1f}% + 個体差{1.5 * sc:.2f}% | {pr * 100:.0f}% |")
        SOURCES["初速（ニップ隙間・スリップ）"]["sigma"] = 0.020
        SOURCES["雑巾の個体差（質量・毛羽）"]["sigma"] = 0.015

        print()
        print("### 厚み計測フィードフォワードで、どこまで詰められるか\n")
        print("シンギュレータのピックローラー変位（重送検知用のホールセンサー）で"
              "1枚ごとの厚みを測り、圧縮率→スリップ率→初速のずれを予測して、"
              "ニップ到達までの 0.33 秒でローラー回転数を補正する。\n")
        base_v = SOURCES["雑巾の個体差（質量・毛羽）"]["sigma"]
        print("| 厚みセンサーの分解能 | 補正後に残る初速 1σ | 命中率 |")
        print("|---|---|---|")
        for res in (1.00, 0.20, 0.05, 0.02):
            resid = feedforward_residual(res)
            SOURCES["雑巾の個体差（質量・毛羽）"]["sigma"] = max(resid, 0.0005)
            SOURCES["初速（ニップ隙間・スリップ）"]["sigma"] = 0.010   # 隙間管理は別途 1% まで詰める前提
            pr, _, _ = run(args.n)
            print(f"| {res:.2f} mm | {resid * 100:.2f}% | {pr * 100:.0f}% |")
        SOURCES["雑巾の個体差（質量・毛羽）"]["sigma"] = base_v
        SOURCES["初速（ニップ隙間・スリップ）"]["sigma"] = 0.020
        print()
        print(f"> 厚みの個体差そのものは 1σ={RAG_T_SIGMA_MM}mm。"
              f"分解能 0.05mm のセンサーなら残差 {feedforward_residual(0.05) * 100:.2f}% まで落ちる。")
        print()
        print("### 対策を積み上げたときの到達点\n")
        print("| 段階 | ニップ隙間 1σ | 個体差 1σ | 命中率 |")
        print("|---|---|---|---|")
        plan = [
            ("対策なし（偏心カム・FFなし）", 0.020, 0.015),
            ("+ 厚みFF（分解能0.2mm）", 0.020, feedforward_residual(0.20)),
            ("+ テーパくさび1:20（分解能3.5µm）", 0.010, feedforward_residual(0.20)),
            ("+ 回転数降下から初速を実測して補正", 0.003, feedforward_residual(0.20)),
        ]
        for label, sg, si in plan:
            SOURCES["初速（ニップ隙間・スリップ）"]["sigma"] = sg
            SOURCES["雑巾の個体差（質量・毛羽）"]["sigma"] = max(si, 0.0005)
            pr, _, _ = run(args.n)
            print(f"| {label} | {sg * 100:.1f}% | {si * 100:.2f}% | **{pr * 100:.0f}%** |")
        SOURCES["初速（ニップ隙間・スリップ）"]["sigma"] = 0.020
        SOURCES["雑巾の個体差（質量・毛羽）"]["sigma"] = 0.015
    else:
        print(f"狙い: v={v0:.2f} m/s  仰角={math.degrees(p0):.1f}°")
        for name, sig, kind, p1, note in rows:
            print(f"  {name:34s} 1σ={sig:<7.4g} 単独命中 {p1 * 100:5.1f}%")
        print(f"\n  全誤差同時: {total * 100:.1f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
