"""全検証を1コマンドで回す.

    python scripts/check_all.py           # 全部走らせて要約（並列）
    python scripts/check_all.py --quick   # 重いもの（シム・干渉）を飛ばす
    python scripts/check_all.py -j 1      # 1本ずつ（出力を追いたいとき）
    python scripts/check_all.py --only 掃引 --only 締結   # 名前で絞る

検証スクリプトが増えて個別に叩くようになり、**回し忘れ**が起きるようになった。
実際、衝突形状の被覆監査（collision_audit.py）は書いてから半日実行していなかった。

`out/*.md` にレポートも同時に書き出す。

並列に回す理由
--------------
検証は 30 本あり、**どれも独立したプロセス**で、書き出す先も別々の
`out/*.md` しかない（共有の生成物を書くのは topo_opt / screw_place だけで、
どちらもこの一覧に入っていない）。逐次で回すと、各プロセスが同じ組立を
1 から作り直す 40 秒を**30 回払う**ことになる。所要時間の合計は
1 本のいちばん長いもの（連続掃引）で決まるので、並列にすればそこまで縮む。
⚠ 1 本あたり 1〜2GB 使う。既定は「コア数の半分」に抑えてあり、
  足りなければ `-j` で下げること（`-j 1` で従来どおりの逐次）。
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PY = os.path.join(ROOT, ".venv", "bin", "python")
OUT = os.path.join(ROOT, "out")

# (表示名, スクリプト, レポート名, 重いか, 合否を判定する語)
#
# ⚠ 判定語は**行頭のマーカー**にすること。本文中の "NG" や "❌" を拾うと、
#   「これはやってはいけない」と示すために**意図的に置いた対照行**まで
#   失敗として数えてしまう。実際そうなった:
#     power_budget  … `[NG運用] 前進しながらスピンアップ 36A` は反面教師の行
#     fork_clearance… 「最悪の重ね合わせ 1.36mm ❌」は最悪ケースの提示
#   どちらも設計は成立している。
# out/ ではなくリポジトリ直下へ書くレポート。**人が読んで発注に使う資料**で、
# `cad/out/*` は .gitignore 対象なので、out/ に置くと履歴に残らない。
ROOT_REPORTS = ("BOM.md", "FAB.md")

# 判定語のかわりに**終了コード**で合否を決める印。
EXIT = "@exit"

CHECKS = [
    ("規定・外形・質量", "scripts/validate.py", "validation.md", True, "| NG |"),
    # **組立の厳格チェック**。判定は距離のしきい値ではなく、部品ごとの
    # 固定宣言（src/tr_fix.py）と実体を突き合わせる。
    #   浮き / 離れ / 宣言のない接触 / 食い込み の 4 つを同時に見る。
    #
    # ⚠ これ以前は「3mm 以上離れていれば OK」で判定していた。
    #   **離れていれば通る基準は、空中に浮いた部品ほど通りやすい。**
    #   実際、STEP には浮いた部品 113 個と食い込み 298 組が同居していた。
    #   旧 interference_full.py / relations_audit.py / consistency.py /
    #   floating.py はこれに置き換えた（残すと「通った」が誤解を生む）。
    ("**組立の厳格チェック**", "scripts/assembly_check.py", "assembly.md", True, "\n### "),
    # **可動軸を連続に見る干渉チェック**。上の assembly_check は 1 姿勢を
    # 厳密に見るが、**姿勢のあいだは見ていない**。
    # ⚠ 姿勢を数点だけ標本にする検査は穴だらけ。grab 316mm を 4 点
    #   （間隔 105mm）、ヨー 60° を 3 点で見ていた。部品の寸法より粗いので
    #   「両端で当たらない」は「途中で当たらない」を意味しない。
    #   実際 press_pad はキャリッジ横材を突き抜けて降りていたのに、
    #   press=0 と 105 では当たらないので長く見逃していた（57,543mm³）。
    # リンクの運動は剛体変換なので、1 回組んだソリッドを変換すれば
    # 組み直し無しで中間姿勢が作れる（ヨー 0.5° / 直動 2mm 刻み）。
    ("**連続掃引の干渉**", "scripts/sweep_fine.py", "sweep_fine.md", True, "\n### "),
    # **連続回転する部品**（車輪・射出ローラー・シンギュレータ）。
    # ⚠ これらは continuous 関節なので**姿勢集合に角度が無い**。そして
    #   軸対称でない（スポーク付きハブ・メカナムのローラー・肉抜き）ので、
    #   ある角度では当たらず別の角度で当たる。45 ソリッドのうち 18 個が
    #   「回すと形が変わる」ことを実体で確かめた。ここを見ていなかった。
    ("**回転部品の全角度**", "scripts/spin_check.py", "spin.md", True, "\n### "),
    # ビューアが「干渉」と表示するものの正体を分けて数える。
    # ⚠ **多くのビューアは面が接していることを干渉として表示する。**
    #   ボルトで留めた面は接しているのが正しいので、正しい組立ほど件数が
    #   増える。いま match で 581 組が「接触（宣言あり）」＝正しい組立。
    #   ここを区別しないと「めっちゃ干渉してる」という見え方になる。
    ("ビューア上の干渉の内訳", "scripts/viewer_report.py", "viewer.md", True, None),
    # URDF は生成物。生成し直して差が無いことを確かめる。
    # ⚠ 生成器がリンクごとに link_*() を直接呼ぶため、同名 put の検査に
    #   引っかかって**生成できない状態で放置されていた**。そのあいだ
    #   grabber_slide の上限（0.4246 ↔ 0.316）や turret_yaw の原点
    #   （0.838 ↔ 0.842）が食い違ったまま残った。制御はこれを見る。
    ("URDF と CAD の一致", "scripts/urdf_check.py", "urdf.md", True, "\n### "),
    # URDF が参照する STL も生成物。同じ理由で生成できず、1 日以上古かった。
    # ⚠ grabber_press.stl は 25 三角形（ただの箱）だった。シミュレーションは
    #   これを見るので「画面では動いているのに実物と違う」ことになる。
    ("STL と CAD の一致", "scripts/mesh_check.py", "mesh.md", True, "\n### "),
    # 連続掃引が「変換だけで足りる」根拠を、組み直して確かめる。
    # ⚠ 姿勢によって形が変わる部品（ブーリアンが空を返して消える、など）が
    #   あると変換では追えない。ブーリアンを走らせず、ソリッド数・部品名・
    #   固定宣言の集合だけ全 46 姿勢で突き合わせる。
    ("姿勢と構成の不変性", "scripts/pose_topology.py", "topology.md", True, "\n### "),
    # 肉抜きした板の**荷重の通り道**が細くなっていないか。
    # ⚠ 「分断」は板が 2 つに割れたときしか落ちない。繋がってさえいれば
    #   幅 1mm の糸でも通るので、荷重を見ずに肉抜きすると「検査は通るのに
    #   実機では裂ける板」ができる。距離変換 + max-min ダイクストラで、
    #   締結の座どうしを結ぶ経路の**いちばん細いところ**を測る。
    ("板の荷重経路の細り", "scripts/ligament_check.py", "ligament.md", True, "\n### "),
    # 投影図。**数値の検査だけでは足りない。**
    # 射出ローラーが旋回テーブルを貫いているような誤りは、図を一度見れば
    # すぐ分かるのに、数値だけで進めていたので長く残っていた。
    # 図そのものは合否を出さないが、毎回描いて残す（out/render_*.png）。
    ("投影図（目視用）", "scripts/render.py", None, False, None),
    ("静解析", "scripts/fea_frame.py", "fea.md", False, "| NG |"),
    ("締結", "scripts/fasteners.py", "fasteners.md", False, "| NG |"),
    # 締結の**員数**。上の fasteners.py は「そのねじで持つか」を見るが、
    # **何本要るか**は誰も数えていなかった。宣言（note）に "4-M4" と
    # 書いてあっても実体のねじは 56 本しか無く、質量台帳は
    # 「ボルト・ナット・スペーサ類 700g」の 1 行だった。
    # ⚠ 判定語は「質量台帳と員数表がずれた」。数え方は tr_lib に 1 つだけ
    #   置いてあるので、ずれたら**どちらかを手で書き換えた**ということ。
    ("締結の員数", "scripts/fastener_bom.py", "fasteners_bom.md", True, "| NG |"),
    # **板の余肉**と**継手の受け方**。どちらも「成立しているが良くない」を
    # 出す検査なので、合否は付けない（判定語 None）。数字が減っているかを
    # 前回の out/ と見比べて使う。
    # ⚠ 台帳の手書き質量と実体の照合もここに入っている。**35kg 規定の
    #   ぎりぎりを狙う設計では、質量表の 1 行の打ち間違いが規定違反そのもの**。
    ("板材の余肉・過大寸法", "scripts/plate_audit.py", "plate_audit.md", True, None),
    ("継手の荷重の受け方", "scripts/joint_load.py", "joint_load.md", True, None),
    # 突き合わせ継手の**内隅**を幾何から数え直す。
    # ⚠ 「ここは L 金具が入らない」という**人の判断**は、図にも台帳にも
    #   残らないまま固定される。マスト上部横梁↔主柱がそうで、註釈には
    #   「柱と横梁の ±X 面は同一平面だから」と書いてあったが、
    #   見ていた面の組が 1 つ足りず、横梁の**下面**と柱の内面という
    #   本来の入隅を丸ごと落としていた（バケツの鉛直荷重の経路）。
    #   判断のほうを毎回検査する。
    ("継手の内隅とL金具", "scripts/corner_bracket.py", "corner_bracket.md", True,
     "NG    "),
    # 板の**外形**をトポロジー最適化した結果は `out/topo/*.json` に残る生成物。
    # ⚠ 生成物なのに、これまでの検査群には「古くなっていないか」を見るものが
    #   1 つも無かった。相手を動かしても板の形だけ前のまま、という状態は
    #   STEP を見ても分からない（形として成立してしまうので）。
    #   鮮度・網羅・作れるか（分断／最小部材幅／座面）を毎回問い直す。
    ("板の最適化輪郭", "scripts/topo_check.py", "topo_check.md", True, EXIT),
    # 締結具の**実体**も `out/screws.json` に残る生成物。
    # ⚠ 宣言（tr_fix）だけあって図にねじが無い状態が長く続いていた
    #   （326 組中 220 組・622 本）。買う数と質量は宣言から見積もれていたが、
    #   頭の座面・工具の入る空間・相手を貫いていないかは実体を置かないと
    #   言えない。位置は凍結してあるので、古くなっていないかを毎回見る。
    ("ねじの自動配置", "scripts/screw_check.py", "screw_check.md", True, "⚠"),
    # ⚠ **置けたことと、留まることは別**。`screw_place` は接触面の有効格子を
    #   頭の座面ぶん収縮してから点を選ぶが、収縮しきった帯では中心線に
    #   寄せるしかない。ここでは凍結した位置を**実体に当てて**、頭の座面が
    #   本当にあるか、下穴の外に 0.3d の壁が残るかを見る。
    #   `edge_tap_check` は注記に「端面」「タップ」と書いた締結しか見ないので、
    #   注記が `2-M4` だけのものはこちらでしか捕まらない。
    ("ねじの座面と肉", "scripts/screw_seat_check.py", "screw_seat.md", True, EXIT),
    ("衝突形状の被覆", "scripts/collision_audit.py", "collision.md", False, None),
    ("公式CADとの照合", "scripts/vendor_check.py", "vendor.md", False, None),
    ("レール取付穴", "scripts/rail_holes.py", "rail_holes.md", False, None),
    ("部材感度", "scripts/frame_optimize.py", "frame_opt.md", True, None),
    ("フォーク隙間", "scripts/fork_clearance.py", "fork.md", False, None),
    ("掃引包絡", "scripts/sweep_envelope.py", "sweep.md", True, None),
    ("サイクルタイム", "scripts/cycle_time.py", "cycle.md", False, "❌"),
    ("電流バジェット", "scripts/power_budget.py", "power.md", False, "| NG |"),
    ("命中率バジェット", "scripts/accuracy_budget.py", "accuracy.md", False, None),
    ("BOM", "scripts/bom.py", "BOM.md", False, None),
    # 製作データ（DXF / STL / STEP）。**生成と検査を分けていない。**
    # ⚠ mesh_check は「temp へ書き出し直して比べる」形にしてあるが、あれは
    #   STL が URDF とシムに**読まれる**生成物だから。製作データは人と
    #   加工屋しか見ないので、毎回 CAD から作り直せば古くなりようがない。
    #   比較のために 245 個の STEP をもう一度書き出すのは、同じ保証を
    #   2 倍の時間で買うだけになる。
    # 落ちるのは「板厚が買えない」「造形範囲を超える」「曲げの展開が要る」
    # ＝**人が決めないと発注できない**もの。
    ("製作データ", "scripts/export_fab.py", "FAB.md", True, "\n## 手が要るもの"),
    # 手順書は**手で書く**ので、CAD を直しても追随しない。BOM は生成物だが
    # ASSEMBLY.md は違う。⚠ 実際こうなっていた:
    #   ・受入れ検査「22本 / 15.97m」← 実際は 30本 / 16.87m
    #   ・側面トラス「L1015 / L810 / L797」← どれも切断リストに無い長さ
    #   ・バケツ上面「1407mm」← 実際 1429mm（MAST_TOP_Z を上げたときの取り残し）
    #   ・工程 0 が「φ26 ではない」と書き、工程 2 が「φ26 を確認」と書いていた
    # 紙を見ながら測る値が違うと、正しい機体のほうを作り直すことになる。
    ("手順書と CAD の一致", "scripts/doc_check.py", "doc.md", True, "\n### 食い違い"),
    # **端面タップの成立性**。曲げ品を「平板 2 枚 + 端面タップ」へ置き換えた
    # 結果、t3 の小口に M4 と書いた締結が 8 組できていた（山が 1.5 山しか
    # 掛からない）。穴は図面どおり開くので**目視でも DXF でも見つからない**。
    ("端面タップ", "scripts/edge_tap_check.py", "edge_tap.md", True, EXIT),
    # 機構シムは **--no-video なら 4.4 秒**。動画込みだと 4〜5分かかるが、
    # そのうち 99% は動画レンダリングで、物理演算は一瞬。
    # 設計の反復では動画を作らない。動画は成果物を見せるときだけ。
    ("機構シム（動画なし）", "sim/tr_sim.py", None, False, "⚠ 机への接触"),
]

# --md を持たないスクリプト（別のフラグで呼ぶ）
ALT_FLAGS = {"sim/tr_sim.py": ["--no-video"], "scripts/render.py": []}


def run(script, report):
    """スクリプトを --md で走らせ、レポートに書き出して stdout を返す。"""
    t0 = time.time()
    flags = ALT_FLAGS.get(script, ["--md"])
    try:
        # ⚠ **1800 秒では足りなくなった。** 板の外形をトポロジー最適化で
        #   切り直したら、輪郭が矩形より複雑になって連続掃引の実体計算が
        #   伸び、`sweep_fine` がここでタイムアウトした（個別に回せば通る）。
        #   検査が「実行エラー」で落ちると、通ったのか落ちたのか分からない
        #   まま先へ進むことになる。時間で切るなら余裕を持つこと。
        r = subprocess.run([PY, script, *flags], cwd=ROOT, capture_output=True,
                           text=True, timeout=5400)
    except subprocess.TimeoutExpired:
        return None, 1800.0, "タイムアウト", 2
    dt = time.time() - t0
    # ⚠ 終了コード 1 は「検査が動いて指摘を出した」の意味。**レポートは書く**。
    #   ここで早期 return していたので、指摘が出た検査ほど out/ に何も残らず、
    #   何が悪いのか読めなかった。実行エラー（2 以上）とは分けて扱う。
    if r.returncode not in (0, 1):
        return None, dt, (r.stderr or r.stdout).strip().splitlines()[-1:] or ["失敗"], r.returncode
    if report:
        # BOM / FAB はリポジトリ直下（**人が読んで発注に使う資料**なので、
        # out/ の生成物とは扱いが違う。`cad/out/*` は .gitignore 対象で、
        # そこへ書くと「発注に使った表」が履歴に残らない）。
        path = (os.path.join(ROOT, report) if report in ROOT_REPORTS
                else os.path.join(OUT, report))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as fp:
            fp.write(r.stdout)
    return r.stdout, dt, None, r.returncode


def verdict(item):
    """1 本走らせて、表示用の（行, 落ちたか）を返す。"""
    name, script, report, _heavy, fail_word = item
    out, dt, err, rc = run(script, report)
    if err:
        return f"{name:<20} {dt:6.1f}s  ⚠ 実行エラー: {err}", True
    # ⚠ 判定語が `EXIT` のものは**その検査の終了コードを合否にする**。
    #   本文の語を数える方式が使えない検査がある。`topo_check` は
    #   一覧表に「NG <指摘>」と書くので、板厚 8mm 未満の穴のような
    #   **情報**でも "NG " が並び、13 枚とも ❌ になっていた。実際には
    #   その回の終了コードは 0（致命も鮮度切れも無い）だった。
    #   合否をどこで決めるかは検査側が知っているので、そちらに従う。
    if fail_word == EXIT:
        return (f"{name:<20} {dt:6.1f}s  " + ("✅" if rc == 0 else "❌ 指摘あり")
                + (f"  → out/{report}" if report else "")), rc != 0
    if fail_word and fail_word in out:
        n = out.count(fail_word)
        return f"{name:<20} {dt:6.1f}s  ❌ {fail_word} が {n} 件", True
    return (f"{name:<20} {dt:6.1f}s  ✅"
            + (f"  → out/{report}" if report else "")), False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="重い検証を飛ばす")
    # ⚠ 既定を「コア数の半分」にしてあるのは**メモリのため**。1 本が
    #   組立を丸ごと持つので 1〜2GB 使う。CPU を埋めきる値にすると、
    #   検証が落ちる代わりに機械ごと詰まる。
    ap.add_argument("-j", "--jobs", type=int,
                    default=max(1, (os.cpu_count() or 4) // 2),
                    help="同時に走らせる本数（既定: コア数の半分。1 で逐次）")
    ap.add_argument("--only", action="append", default=[],
                    help="表示名にこの文字列を含むものだけ走らせる（複数可）")
    args = ap.parse_args()

    todo, skipped = [], []
    for c in CHECKS:
        name, _script, _report, heavy, _fw = c
        if args.quick and heavy:
            skipped.append(f"{name:<20} {'—':>7}  スキップ（--quick）")
        elif args.only and not any(k in name for k in args.only):
            continue
        else:
            todo.append(c)
    # ⚠ **重いものから始める**。所要時間の合計は最後に終わる 1 本で決まる
    #   ので、重いものを後ろに回すと、軽いものが全部終わったあとに
    #   1 本だけ走っている時間が伸びる（連続掃引だけで 10 分以上ある）。
    todo.sort(key=lambda c: not c[3])

    print(f"{'検証':<20} {'時間':>7}  結果"
          + (f"    （{args.jobs} 並列・終わった順）" if args.jobs > 1 else ""))
    print("-" * 62)
    for line in skipped:
        print(line)

    bad = 0
    t0 = time.time()
    if args.jobs > 1:
        # ⚠ 走らせるのは別プロセスなので、待つ側はスレッドで足りる。
        # ⚠ `map` ではなく `as_completed` で受ける。`map` は**投入した順**に
        #   返すので、重いものを先頭に置いた今の並びだと、軽いものが全部
        #   終わっていても画面には何も出ない（10 分無反応に見える）。
        with ThreadPoolExecutor(max_workers=args.jobs) as ex:
            futs = [ex.submit(verdict, item) for item in todo]
            for fut in as_completed(futs):
                line, ng = fut.result()
                print(line, flush=True)
                bad += ng
    else:
        for item in todo:
            line, ng = verdict(item)
            print(line, flush=True)
            bad += ng
    print("-" * 62)
    print(f"実時間 {time.time() - t0:.1f}s")
    print("すべて通った ✅" if bad == 0 else f"⚠ {bad} 件で問題あり")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
