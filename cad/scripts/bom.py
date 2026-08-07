"""TR 部品表（BOM）と加工リストを CAD から生成する.

    python scripts/bom.py            # 人間可読
    python scripts/bom.py --md       # Markdown（BOM.md へ貼る用）

出力するもの
  1. アルミフレーム切断リスト（長さごとに集約 → そのまま MISUMI へ指定長発注できる）
  2. CNC切削板・板金リスト（材質と板厚ごと）
  3. 購入品リスト（型番・数量・単価・小計）と予算 40 万円チェック
  4. 締結部品の概算

購入品の単価は 2026-07 時点の実勢の概算。発注前に必ず再見積もりすること。
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from collections import defaultdict

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

import tr_assembly as A  # noqa: E402
import tr_lib as L  # noqa: E402
import tr_params as P  # noqa: E402

# 購入品: (品名, 型番/仕様, 数量, 単価[円], 予算対象か)
PURCHASED = [
    ("ブラシレスモーター", "DJI RoboMaster M3508 P19 + C620 ESC", 7, 18000, True),
    ("ブラシレスモーター", "DJI RoboMaster M2006 P36 + C610 ESC", 4, 7000, True),
    ("メカナムホイール", "φ100 アルミハブ 9ローラー（左右各2）", 4, 8000, True),
    # ⚠ **ここが予算を割る。** 北陽 UST-20LX は 1 台 ¥243,500（税抜／
    #   実売 ¥267,900 税込）で、3 台で ¥730,500。規定 3.1.6 の新規調達枠
    #   40 万円をこれ 1 品目で超える。設計としては UST-20LX 想定で進めるが、
    #   **調達は「手持ち品を使う（＝新規調達ではない）」か「低価格機に
    #   置換する」かを決める必要がある**。置換するなら LD19 / STL-19P 等の
    #   クラス1 機（1 台 ¥7,000）で、外形 50 角・検出面 47.4 の前提が変わる
    #   （`tr_params.LIDAR_*` を実測値に入れ替えること）。
    ("2D LiDAR", "北陽電機 UST-20LX（クラス1・20m・270°・50×50×70）",
     3, 243500, True),
    ("スライドレール", f"MISUMI {P.RAIL_MODEL}（3段引・中荷重・スチール）", 2, 1435, True),
    ("アルミフレーム", "MISUMI HFS5-2020 指定長（切断リスト参照）", 1, 9000, True),
    ("フレーム用ブラケット/ナット/キャップ", "MISUMI 5シリーズ", 1, 6000, True),
    ("旋回支持", "Vリング φ220 + カムフォロア 3個", 1, 6000, True),
    # ヨー駆動（20T/60T）＋ グラバーのベルト搬送
    ("タイミングプーリ/ベルト（ヨー・搬送）", "HTD5M 20T/60T + φ40プーリ + ベルト",
     1, 9000, True),
    # 射出ローラー駆動。**モーターを軸の延長線上に置けない**ため後付けした
    # （軸端に付けると先端が |Y|=408 になり側面上桁 340〜360 を貫通する）。
    ("タイミングプーリ 24T ×4（ローラー駆動）", "HTD5M 24T φ38.2 幅15", 4, 1800, True),
    ("タイミングベルト ×2（ローラー駆動）", "HTD5M 幅15 軸間85mm", 2, 1500, True),
    # ⚠ ウォームとホイールは**歯切りが要る**（KHK SW1.5-1 相当 + G1.5-40 相当）。
    #   自校で削れるのはブランクまでなので、歯は買うか外注する前提で見積もる。
    ("ウォーム減速（仰角）", "ウォーム m1.5 1条 φ24 + ホイール m1.5 z40 φ63（40:1）",
     1, 8000, True),
    ("ウォーム軸まわり", "6001ZZ ×2 + スラストカラー ×2 + φ8-φ12 カップリング",
     1, 3000, True),
    ("軸受・軸・止め輪", "深溝玉軸受 各種 + φ8〜φ12 軸", 1, 12000, True),
    ("CNC切削材料", "A5052 t2/t3/t4/t5/t6 板材（自校加工前提）", 1, 25000, True),
    ("板金材料", "A5052 t1.0/t1.5 + SUS304 t2.0", 1, 9000, True),
    ("3Dプリント材料", "PETG フィラメント + ウレタンシート t3", 1, 8000, True),
    ("制御基板", "STM32 系 CAN 制御基板（自作 or RoboMaster 開発ボード）", 1, 15000, True),
    ("配線・コネクタ・端子", "XT60 / CAN / シールド線 一式", 1, 12000, True),
    ("安全部品", "主幹30Aブレーカー + 非常停止SW×2 + 電源LED", 1, 9000, True),
    ("ポリカーボネート", "t1.0 スカート・カバー", 1, 6000, True),
    # ⚠ 完成品の「オドメトリポッド」は買わない（φ50・幅 12・片持ち・
    #   |Y|=350 で外側 4mm という条件に合う市販品が無い）。
    #   取付板とキャリッジ金具は PETG（3Dプリント材料に計上）、購入するのは
    #   軸受・エンコーダ・磁石・オムニ・リニアガイド・輪ゴム。
    #   1 輪あたりの内訳（2026-08-07 に構成変更 → DESIGN §45）:
    #     双列オムニ φ50×W20 ¥1,600 / リニアガイド MGN9 L64 + MGN9C ¥1,300 /
    #     MR105ZZ ×2 ¥300 / AS5600 基板 ¥500 / 磁石 φ6×2 ¥200 /
    #     輪ゴム #10（袋・予備込み）¥100
    #   ⚠ **輪ゴムは消耗品**。試合ごとに掛け替えるので、1 袋では足りない
    #     ことを前提に多めに見ておく（金額としては誤差）。
    ("計測輪 購入部品",
     "双列オムニφ50 / MGN9+MGN9C / MR105ZZ / AS5600基板 / 磁石 / 輪ゴム#10",
     3, 4000, True),
    # LiDAR のレベリング座（3 台 × 4 点）。板とねじは切削材料/ねじ一式に
    # 入るので、ここで買うのは**ばねだけ**。1 台あたり 4 個。
    ("レベリング座 圧縮ばね",
     f"φ{P.LIDAR_LVL_SPRING_OD:.0f}×{P.LIDAR_LVL_SPRING_FREE:.0f} "
     f"素線φ0.8 k={P.LIDAR_LVL_SPRING_K:.1f}N/mm", 12, 100, True),
    ("ToF測距センサ", "机エッジ検出用 ×2", 2, 2500, True),
    ("IMU", "6軸/9軸 モジュール", 1, 3000, True),
    ("ねじ・ナット・スペーサ", "M3/M4/M5 一式", 1, 12000, True),
    # --- 予算対象外（規定 3.1.6） ---
    ("バッテリー", "6S LiPo 22.2V 3000mAh ×2", 2, 9000, False),
    ("コントローラ/無線", "プロポ + 2系統無線モジュール", 1, 30000, False),
    ("椅子", "新JIS 教室用椅子 5号", 1, 6000, False),
    ("移動バケツ", "エンテック PO-24A 10L ×2（予備込み）", 2, 900, False),
    # マスコット「シボリ」（規定 3.1.3・重量制限外）。形は `L.mascot_parts`。
    #   芯 EPP ブロック 20L 相当 / 表面フェルト / つなぎ・三角巾の生地 /
    #   ボタン / 尻に埋める M6 鬼目ナット 2 + 蝶ボルト（工具無しで外せる）
    ("マスコット材料", "EPP ブロック + フェルト + つなぎ/三角巾 + M6鬼目ナット",
     1, 10000, False),
]


def cut_list(series: str = "HFS5-2020"):
    """アルミフレームの切断リスト（長さで集約）。

    ⚠ **型番を決め打ちにしない。** `HFS5-2020` だけを拾っていたので、
      側面斜材の **HFS5-2010 が 1 本も表に出ていなかった**（`add_ext2010`
      で台帳には載っている）。この表を見て発注すると、斜材が 4 本まるごと
      届かない。型番ごとに `cut_lists()` で全部出すこと。
    ⚠ **員数 0 の行は出さない。** 廃止した砲塔横梁が `qty=0` で台帳に
      残してあり（消すと「なぜ無いのか」が失われるので残すのは正しい）、
      切断リストに「長さ 0 / 0 本」の行が出ていた。買う相手に渡す表なので、
      買わないものは載せない。
    """
    A.build(P.POSE_MATCH)
    cuts: dict[int, list[str]] = defaultdict(list)
    for item in L.LEDGER.items:
        m = re.match(re.escape(series) + r" (.+) L(\d+)$", item.label)
        if m and item.qty > 0 and int(m.group(2)) > 0:
            cuts[int(m.group(2))].extend([m.group(1)] * item.qty)
    return dict(sorted(cuts.items(), reverse=True))


def cut_lists():
    """台帳に出てくる押出材の型番ごとに切断リストを返す。"""
    A.build(P.POSE_MATCH)
    series = sorted({m.group(1) for it in L.LEDGER.items
                     if (m := re.match(r"(HFS5-\d+) .+ L\d+$", it.label))})
    return {s: cut_list(s) for s in series}


def plate_list():
    """切削板・板金の一覧（ラベルから材質と板厚を拾う）。"""
    A.build(P.POSE_MATCH)
    out = []
    for item in L.LEDGER.items:
        if "A5052" in item.label or "SUS304" in item.label or "ポリカ" in item.label:
            out.append((item.label, item.qty, item.unit_kg))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--md", action="store_true")
    args = ap.parse_args()
    md = args.md

    lists = cut_lists()
    cuts = lists.get("HFS5-2020", {})
    total_len = sum(k * len(v) for k, v in cuts.items())
    n_cut = sum(len(v) for v in cuts.values())
    # 1m あたり質量 [kg]。⚠ 2010 は 2020 の半分（断面が半分）。
    KG_PER_M = {"HFS5-2020": P.EXT_MASS_PER_M, "HFS5-2010": 0.250}

    if md:
        print("## 1. アルミフレーム切断リスト（MISUMI 指定長）\n")
        for series, cl in lists.items():
            if not cl:
                continue
            n = sum(len(v) for v in cl.values())
            ln_sum = sum(k * len(v) for k, v in cl.items())
            print(f"### {series}\n")
            print("| 長さ [mm] | 本数 | 用途 |")
            print("|---|---|---|")
            for ln, names in cl.items():
                uses = ", ".join(sorted(set(names)))
                print(f"| {ln} | {len(names)} | {uses} |")
            kg = ln_sum / 1000 * KG_PER_M.get(series, P.EXT_MASS_PER_M)
            print(f"\n{series} 小計 **{n} 本 / {ln_sum / 1000:.2f} m**"
                  f"（質量 {kg:.2f} kg）\n")
        n_all = sum(sum(len(v) for v in cl.values()) for cl in lists.values())
        m_all = sum(sum(k * len(v) for k, v in cl.items())
                    for cl in lists.values()) / 1000
        print(f"押出材 合計 **{n_all} 本 / {m_all:.2f} m**\n")
        print("> MISUMI の指定長は 0.5mm 単位・公差±0.5mm。端面タップ（LTP/RTP）を併用すると"
              "ブラインドジョイントで組め、ブラケットが減って軽くなる。\n")
        print("## 2. 切削板・板金リスト\n")
        print("| 部品 | 数量 | 単重 [kg] |")
        print("|---|---|---|")
        for label, qty, kg in plate_list():
            print(f"| {label} | {qty} | {kg:.3f} |")
        print("\n## 3. 購入品と予算（規定 3.1.6：新規調達 40万円以内）\n")
        print("| 品目 | 型番/仕様 | 数量 | 単価 | 小計 | 予算対象 |")
        print("|---|---|---|---|---|---|")
        budget = 0
        outside = 0
        for name, spec, qty, price, in_budget in PURCHASED:
            sub = qty * price
            if in_budget:
                budget += sub
            else:
                outside += sub
            print(f"| {name} | {spec} | {qty} | ¥{price:,} | ¥{sub:,} | "
                  f"{'○' if in_budget else '—'} |")
        print(f"| **予算対象 合計** | | | | **¥{budget:,}** | 上限 ¥400,000 "
              f"{'✅' if budget <= 400000 else '❌'} |")
        print(f"| 予算対象外 合計 | 電池・コントローラ・椅子・バケツ・マスコット | | | "
              f"¥{outside:,} | — |")
    else:
        print(f"アルミフレーム: {n_cut} 本 / {total_len / 1000:.2f} m")
        for ln, names in cuts.items():
            print(f"  L{ln:>5} × {len(names):2d}  {', '.join(sorted(set(names)))[:60]}")
        print(f"\n切削板・板金: {len(plate_list())} 種")
        budget = sum(q * p for _, _, q, p, b in PURCHASED if b)
        outside = sum(q * p for _, _, q, p, b in PURCHASED if not b)
        print(f"\n購入品 予算対象 ¥{budget:,} / 上限 ¥400,000 "
              f"{'OK' if budget <= 400000 else 'NG'}   （対象外 ¥{outside:,}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
