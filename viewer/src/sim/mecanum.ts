/**
 * メカナム逆運動学: 車体速度 → 各輪角速度 (rad/s)
 * strategy.md §4.4/§4.5.3。ホイール順: [FL, FR, RL, RR]
 * theta: 車体前方 = (sinθ, cosθ) (z+ 方向が θ=0)
 */
export function wheelOmegas(
  theta: number,
  vx: number,
  vz: number,
  omega: number,
  r: number,
  k: number,
): [number, number, number, number] {
  const fx = Math.sin(theta);
  const fz = Math.cos(theta);
  const rx = Math.cos(theta);
  const rz = -Math.sin(theta);
  const vf = vx * fx + vz * fz; // 前後
  const vs = vx * rx + vz * rz; // 右ストレイフ
  return [
    (vf - vs - k * omega) / r,
    (vf + vs + k * omega) / r,
    (vf + vs - k * omega) / r,
    (vf - vs + k * omega) / r,
  ];
}

/** 車体右方向の速度成分 (樽ローラーの空転アニメーション用) */
export function strafeSpeed(theta: number, vx: number, vz: number): number {
  return vx * Math.cos(theta) - vz * Math.sin(theta);
}
