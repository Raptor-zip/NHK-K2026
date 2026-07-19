/**
 * モーター × 射出機構の適合マトリクス。各機構が駆動軸に要求する (rpm, トルク, 電力) を
 * 解析計算し、手持ちの各モーター(24V)で「何個で成立するか」を判定して表にする。
 *   npx tsx scripts/rag-motors.ts
 * 出力: コンソール表 + out/rag/motors.md
 * ※スペックはデータシート/公称の実測値。BLDCのKtは Kt≈9.5488/Kv。
 */
import { mkdir, writeFile } from 'node:fs/promises';
import { DEFAULT_PARAMS } from '../src/config/params';

const RHO = 1.225;
const CD = 1.5;
const AREA = 0.2 * 0.3;
const ragM = DEFAULT_PARAMS.rag.m;

interface Motor {
  name: string;
  maxRpm: number; // 24Vでの実用最大回転(出力軸)
  kt: number; // トルク定数 N·m/A (出力軸)
  imax: number; // 短時最大電流 A
  pmax: number; // 実用最大出力 W
  mass: number; // kg
  peakTorque: number; // ピークトルク N·m (出力軸)
  control: string; // 制御/ドライバ
  source: string; // 出典URL
  use: string; // 推奨用途
  note: string;
}
// 実測/公称 (24V)。BLDC(5065/4250)のKtは Kt≈9.5488/Kv。Kvは代表値(実機で要確認)。
const MOTORS: Motor[] = [
  { name: 'M3508 直結(ギヤ無)', maxRpm: 9250, kt: 0.0156, imax: 20, pmax: 200, mass: 0.365, peakTorque: 0.23, control: 'C620 ESC / CAN・FOC・エンコーダ内蔵', source: 'https://www.robomaster.com/en-US/products/components/general/M3508', use: 'ベルト/ローラー(直結×2)', note: '高回転・低トルク(P19ギヤを外した場合)' },
  { name: 'M3508 P19(19.2:1)', maxRpm: 469, kt: 0.3, imax: 20, pmax: 200, mass: 0.365, peakTorque: 4.5, control: 'C620 ESC / CAN・FOC・エンコーダ内蔵', source: 'https://www.robomaster.com/en-US/products/components/general/M3508', use: 'スリング/投石機の巻上げ(×1)', note: '低回転・高トルク(ギヤ付の標準形)' },
  { name: 'CyberGear(7.75:1)', maxRpm: 296, kt: 0.87, imax: 14, pmax: 200, mass: 0.317, peakTorque: 12, control: 'CAN・FOC・14bitエンコーダ内蔵(MITモード)', source: 'https://wiki.openelab.io/motors/cybergear-micromotor-instruction-manual', use: '遠心スリング(×1, ギヤ不要)', note: 'QDDアクチュエータ。高トルク・低速' },
  { name: '5065 270Kv', maxRpm: 6480, kt: 0.0354, imax: 80, pmax: 1550, mass: 0.45, peakTorque: 2.83, control: '要 VESC等(標準センサレス, Hall付版あり)', source: 'https://www.esk8.store/product/flipsky-270kv-8s-1550w-5065-bldc-motor-for-diy-electric-skateboard-ebike/', use: 'ベルト/ローラー(×1)', note: '高速・大電力・トルク余裕大' },
  { name: '4250 540Kv', maxRpm: 12960, kt: 0.0177, imax: 52, pmax: 800, mass: 0.21, peakTorque: 0.92, control: '要 ESC(RC用, センサレス)', source: 'https://www.amazon.com/4250-KV540-Brushless-Outrunner-Motor-540kV/dp/B00VR5LCKO', use: 'ベルト/ローラー(×1, 軽量)', note: '超高速・軽量。Kv要確認(410〜800の版あり)' },
];

interface Req {
  key: string;
  label: string;
  rpm: number;
  torque: number;
  power: number;
}
function mechRequirements(): Req[] {
  // 遠心スリング(yaw/pitch): 腕を ω=16rad/s へ180°加速
  const ARM_LEN = 0.5;
  const TETHER = 0.12;
  const OMEGA = 16;
  const ARM_MASS = 0.25;
  const rEff = ARM_LEN + TETHER;
  const I = (1 / 3) * ARM_MASS * ARM_LEN * ARM_LEN + ragM * rEff * rEff;
  const alpha = (OMEGA * OMEGA) / (2 * Math.PI);
  const tauAero = 0.5 * RHO * CD * AREA * (OMEGA * rEff) ** 2 * rEff;
  const slingTorque = I * alpha + tauAero;
  const sling: Req = { key: 'sling', label: '遠心スリング(yaw/pitch)', rpm: (OMEGA * 60) / (2 * Math.PI), torque: slingTorque, power: slingTorque * OMEGA };

  // 対向ベルト(長ニップ・対称): プーリr20mm, ニップ0.35m。布をベルト面速度まで摩擦加速。
  // 速度はD=3mバケツ最適(rag-optimize-bucket)の動作点。要求は排出速度^2に比例(遠距離ほど増)。
  const pulleyR = 0.02;
  const carriage = 0.15; // ベルト+プーリの実効慣性
  const beltExit = 4.5;
  const beltNip = 0.35;
  const beltForce = (ragM + carriage) * ((beltExit * beltExit) / (2 * beltNip)) + 0.5 * RHO * CD * AREA * beltExit * beltExit;
  const belt: Req = { key: 'belt', label: '対向ベルト式(長ニップ)', rpm: (beltExit / (2 * Math.PI * pulleyR)) * 60, torque: beltForce * pulleyR, power: beltForce * beltExit };

  // 2段対向ベルト(短ニップ・差動でバックスピン): プーリr20mm, ニップ0.18m。D=3m最適動作点。
  const rollerExit = 4;
  const rollerNip = 0.18;
  const rollerForce = (ragM + carriage) * ((rollerExit * rollerExit) / (2 * rollerNip)) + 0.5 * RHO * CD * AREA * rollerExit * rollerExit;
  const roller: Req = { key: 'roller', label: '2段対向ベルト式(差動)', rpm: (rollerExit / (2 * Math.PI * pulleyR)) * 60, torque: rollerForce * pulleyR, power: rollerForce * rollerExit };

  // 投石機の装填(コッキング): 射出は重力だが、毎射CWを持ち上げアームを起こすモーターが要る。低速・高トルク。
  const cwKg = 0.8; // D=3m最適のCW
  const cockArmShort = 0.18;
  const cockAngle = (160 * Math.PI) / 180;
  const cockTime = 2.0; // 装填時間 s
  const omegaCock = cockAngle / cockTime;
  const cockTorque = cwKg * 9.81 * cockArmShort + 0.25 * 9.81 * (0.6 / 2); // CW持ち上げ + 長腕自重概算
  const trebuchet: Req = { key: 'trebuchet', label: '投石機の装填(コッキング)', rpm: (omegaCock / (2 * Math.PI)) * 60, torque: cockTorque, power: cockTorque * omegaCock };

  return [sling, trebuchet, belt, roller];
}

function assess(r: Req, m: Motor): { motors: number; ok: boolean; over: boolean; iPer: number } {
  if (r.rpm > m.maxRpm * 1.03) return { motors: Infinity, ok: false, over: true, iPer: 0 };
  const nT = Math.ceil(r.torque / (m.kt * m.imax));
  const nP = Math.ceil(r.power / m.pmax);
  const motors = Math.max(1, nT, nP);
  return { motors, ok: motors <= 3, over: false, iPer: r.torque / motors / m.kt };
}

const reqs = mechRequirements();

// ---- コンソール表 ----
console.log('\nモーター×機構 適合判定 (24V, 「Nコで成立」/ ✕=回転数超過or要4個以上)\n');
const head = '機構\\モーター'.padEnd(26) + MOTORS.map((m) => m.name.padEnd(20)).join('');
console.log(head);
console.log('-'.repeat(head.length));
for (const r of reqs) {
  const row =
    `${r.label} (${r.rpm.toFixed(0)}rpm/${r.torque.toFixed(2)}N·m/${r.power.toFixed(0)}W)`.padEnd(26) +
    MOTORS.map((m) => {
      const a = assess(r, m);
      const cell = a.over ? '✕ 回転超過' : `${a.ok ? '○' : '✕'}×${a.motors}(${a.iPer.toFixed(0)}A)`;
      return cell.padEnd(20);
    }).join('');
  console.log(row);
}

// ---- Markdown: 使えるモーター一覧 + 適合表 ----
const md: string[] = [];
md.push('# 使えるモーター一覧 — 雑巾射出機構向け (24V想定)\n');
md.push('_`viewer/scripts/rag-motors.ts` が生成。スペックはデータシート/公称の実測値、BLDCの Kt≈9.5488/Kv。Kvは代表値なので実機で要確認。_\n');

md.push('## 手持ちモーター諸元\n');
md.push('| モーター | 最大rpm | ピークτ | Kt [N·m/A] | 最大電流 | 最大出力 | 質量 | 制御/ドライバ | 推奨用途 |');
md.push('|---|---|---|---|---|---|---|---|---|');
for (const m of MOTORS)
  md.push(`| ${m.name} | ${m.maxRpm} | ${m.peakTorque} N·m | ${m.kt} | ${m.imax}A | ${m.pmax}W | ${(m.mass * 1000).toFixed(0)}g | ${m.control} | ${m.use} |`);
md.push('\n出典: ' + MOTORS.map((m) => `[${m.name.split('(')[0].trim()}](${m.source})`).filter((v, i, a) => a.indexOf(v) === i).join(' / '));

md.push('\n## モーター × 機構 適合判定\n');
md.push('○=1〜3個で成立 / ✕=回転数超過 or 4個以上必要。( )内は各モーターの電流。\n');
md.push('| 機構 (要求 rpm / τ / W) | ' + MOTORS.map((m) => m.name).join(' | ') + ' |');
md.push('|---|' + MOTORS.map(() => '---').join('|') + '|');
for (const r of reqs) {
  const cells = MOTORS.map((m) => {
    const a = assess(r, m);
    return a.over ? '✕ 回転超過' : `${a.ok ? '○' : '✕'} ×${a.motors} (${a.iPer.toFixed(0)}A)`;
  });
  md.push(`| ${r.label} (${r.rpm.toFixed(0)} / ${r.torque.toFixed(2)} / ${r.power.toFixed(0)}) | ${cells.join(' | ')} |`);
}

md.push('\n## 読み取り・推奨');
md.push('- **対向ベルト / 2段対向ベルト(差動)**: D=3mバケツ最適は低速(排出4〜4.5m/s≒2000rpm・0.1〜0.2N·m・30〜40W)。**M3508直結×1で成立**(各9〜12A, C620の20A内)。P19ギヤを外す方針にそのまま合致。5065×1(4〜5A)や4250×1でも可。CyberGear/P19は回転数超過で不適。');
md.push('- **CyberGear**: 高トルク・低速(296rpm上限)。**遠心スリングを1個で駆動**(6A)。M3508でギヤが要った低速高トルク域をカバー。回転数が要るベルトには不適。');
md.push('- **M3508**: **直結(ギヤ無)がベルト/2段ベルトに最適(各×1)** — 低速なので直結の高回転が余裕、C620でCAN・FOC制御が容易。P19ギヤ(19.2:1)は遠心スリング/巻上げ(低速高トルク)向き(×1, 17A)。');
md.push('- **5065 / 4250**: 高速・大電力で余裕は大きいが、この用途(低速3m)では過剰。要 VESC 等の外部ESC(M3508/CyberGearはドライバ内蔵で制御が楽)。遠距離・高速射出に振るなら優位。');
md.push('- **制御の実務**: M3508(C620)・CyberGear は CAN・FOC・エンコーダ内蔵で位置/速度/トルク制御が容易。5065/4250 は生のRCモーターで VESC 等の外部ESC(＋フライホイールなら開ループ可)が要る。');
md.push('- **結論**: **ベルト/2段ベルト路線 → M3508直結×1**(P19を外して直結, 制御容易, 手持ちで完結)。**遠心スリング路線 → CyberGear×1**(ギヤ不要・高トルク・制御容易)。いずれも手持ちでカバー可能。');
md.push('\n> 注: 要求値は「D=3mバケツ最適(rag-optimize-bucket)」の動作点。**要求トルク/電力は排出速度^2にほぼ比例**するので、遠距離狙いで速度を上げると急増する(短ニップの2段式は特に顕著)。`scripts/rag-mech.ts`冒頭の定数と対応。');

const docsDir = new URL('../docs/', import.meta.url).pathname;
await mkdir(docsDir, { recursive: true });
await writeFile(`${docsDir}motors.md`, md.join('\n') + '\n');
// 生成物側にもコピー(既存の参照互換)
const outDir = new URL('../out/rag/', import.meta.url).pathname;
await mkdir(outDir, { recursive: true });
await writeFile(`${outDir}motors.md`, md.join('\n') + '\n');
console.log(`\n📄 viewer/docs/motors.md (追跡用) と out/rag/motors.md に保存`);
