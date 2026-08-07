"""TR 機構解析シミュレーション（PyBullet）.

`cad/urdf/tr.urdf` をそのまま読み込み、実際の競技オブジェクト（補充スポットの机・
雑巾の山・教壇・旗・固定バケツ）を置いて、装填 → 走行 → 射出の一連を動かす。

目的は2つ:
  1. 「組み立てたら動く」ことの確認 — 関節の可動域・干渉・順序が成立するか
  2. 機構解析 — 関節反力/トルク、重心移動、フォーク先端の軌跡を数値で取る

割り切り（意図的な簡略化）
  * メカナムの横滑りは PyBullet の摩擦モデルでは正しく出ないため、車体は
    「仮想ホロノミック駆動」= 目標軌道への PD 力/トルクで動かす。車輪は接地して
    荷重を支え、関節は見た目の回転速度に追従させる。装填系・砲塔は完全に動力学。
  * 雑巾は剛体プレート（200×300×3mm, 48g）で近似する。布の空力・はためきは
    `gpusim/` の専用シムが担当するので、ここでは機構との干渉と射出初期条件だけ見る。

    python sim/tr_sim.py --video out/sim/tr_mission.mp4
    python sim/tr_sim.py --no-video          # 数値だけ速く回す
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys

import numpy as np
import pybullet as p
import pybullet_data

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

import tr_params as P  # noqa: E402

sys.path.insert(0, os.path.join(HERE, "..", "control"))
import tr_control as C  # noqa: E402

MM = 0.001
URDF = os.path.abspath(os.path.join(HERE, "..", "urdf", "tr.urdf"))

DT = 1.0 / 240.0
FPS = 30
SUBSTEPS = int(round(1.0 / FPS / DT))

# --- 競技オブジェクト配置（ローカル基準: ロボットは +X を向いて射出する） -----
DESK_X = -1.20          # 補充スポット机の中心 X
DESK_TOP = P.DESK_H * MM
STACK_N = 10            # 積層された雑巾の枚数
STACK_EDGE = 0.030      # 山を机の手前エッジから何m内側に置くか（人間が整える標準形）
STACK_LIFT = 0.0050     # 山の初期浮き。剛体プレートでは櫛歯のくさびが布端を持ち上げる
                        # 挙動を再現できないため、その分だけ最初から浮かせて代用する
APPROACH_X = -0.500     # 机の前脚から50mm手前で止まる車体位置（天板下へ潜り込む）
PODIUM_X = 1.10         # 教壇（W600×H200、Y方向に伸びる）
FLAG_X = 3.30           # 旗ポール
FLAG_BAR_Z = P.FLAG_CROSSBAR_Z * MM
BUCKET_FIXED = [(0.60, -1.40, 0.0), (1.30, 1.45, 0.60), (-0.40, 1.45, 0.30)]

# --- ミッション（時刻[s], フェーズ名, 目標） --------------------------------
MISSION = [
    (0.0, "初期化", dict(base_x=0.0, pitch=P.PITCH_DEFAULT, slide=0.0, press=0.0, roller=0.0)),
    (1.5, "補充スポットへ後退", dict(base_x=APPROACH_X)),
    (4.5, "フォーク挿入", dict(slide=P.GRAB_STROKE * MM)),
    (6.5, "上押さえ下降", dict(press=P.PRESS_STROKE * MM)),
    (7.5, "山ごと引込（押さえ保持）", dict(slide=0.050)),
    (9.4, "押さえ解放", dict(press=0.0)),
    (10.2, "最終引込→カム傾斜→ホッパー投入 / 停止中にローラースピンアップ",
     dict(slide=0.0, roller=110.0)),
    (11.6, "射撃位置へ前進（ローラーは定速維持）",
     dict(base_x=0.30, roller=144.0, pitch=54.0)),
    (14.5, "射出1", dict(fire=1)),
    (17.0, "射出2", dict(fire=1, yaw=8.0)),
    (19.5, "射出3", dict(fire=1, yaw=-8.0)),
    (22.0, "終了", dict()),
]
T_END = 24.0
CAM_START = 0.080       # この引込量から下でカムが櫛歯を押し下げ始める
CAM_END = 0.020

MUZZLE_SPEED = 6.5      # m/s（旗3.9m・仰角54°。布の揚力補正込みの実効指令値）
ABORT_PUSH = 0.005      # m 山をこれ以上押したら挿入を中止して引き戻す
                        # （実機ではフォーク根元のリミットSWが抵抗を検知する）
CRUISE_SPEED = 0.80     # m/s 走行指令の速度上限
ACCEL_MAX = 1.5         # m/s^2 電流バジェット内の設計加速度（減速も同じ）


def _q(*e):
    return p.getQuaternionFromEuler(e)


class Sim:
    CRUISE = CRUISE_SPEED

    def __init__(self, gui: bool = False, cam: str = "wide", perturb: dict | None = None):
        self.cam = cam
        # 実戦のばらつき: 山の置き位置・机の位置・車体の停止位置
        self.pert = dict(stack_dx=0.0, stack_dy=0.0, stack_yaw=0.0,
                         desk_dx=0.0, desk_dy=0.0, base_err=0.0)
        if perturb:
            self.pert.update(perturb)
        self.cid = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setPhysicsEngineParameter(fixedTimeStep=DT, numSolverIterations=80)
        p.setTimeStep(DT)
        self.plane = p.loadURDF("plane.urdf")
        p.changeDynamics(self.plane, -1, lateralFriction=0.9)
        self.stack_spawn = []
        self._build_field()
        self._load_robot()
        self.rags = []
        self.log = []
        # 走行指令・逆運動学・インターロックは control/tr_control.py の実装をそのまま使う。
        # → このシミュレーションが「実機に載せる制御ロジック」の検証になる。
        self.profile = C.TrapezoidProfile(cruise=CRUISE_SPEED, accel=ACCEL_MAX)
        self.mk = C.MecanumKinematics()
        self.interlocks = C.Interlocks()
        self.cmd_x = 0.0
        self.violations = []
        self.t_now = 0.0
        self.fouls = []
        self._clamped = False
        self._grasp_cons = []
        self.grasped = 0
        self.pushed_mm = 0.0
        self.aborted = False
        self.abort_t = None
        self.push_at_abort = 0.0

    # ------------------------------------------------------------------ field
    def _box(self, half, pos, orn=(0, 0, 0, 1), mass=0.0, rgba=(0.8, 0.8, 0.82, 1)):
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba)
        return p.createMultiBody(mass, col, vis, pos, orn)

    def _build_field(self):
        w, d, h = P.DESK_W * MM, P.DESK_D * MM, P.DESK_H * MM
        dk_x = DESK_X + self.pert["desk_dx"]
        dk_y = self.pert["desk_dy"]
        # 机（天板＋脚）は **1つの剛体** として作る。
        # 拘束でつなぐと、ロボットが触れていなくてもソルバーの残差で数mm動いてしまい、
        # 「机を動かしたか（4.1.4a）」の判定が濁る。
        leg_h = (h - 0.02) / 2
        poss = [[0, 0, h - 0.009]]
        for sx in (-1, 1):
            for sy in (-1, 1):
                poss.append([sx * (d / 2 - 0.03), sy * (w / 2 - 0.03), leg_h])
        self.desk = p.createMultiBody(
            8.0,
            p.createCollisionShapeArray(
                [p.GEOM_BOX] * 5,
                halfExtents=[[d / 2, w / 2, 0.009]] + [[0.015, 0.015, leg_h]] * 4,
                collisionFramePositions=poss),
            p.createVisualShapeArray(
                [p.GEOM_BOX] * 5,
                halfExtents=[[d / 2, w / 2, 0.009]] + [[0.015, 0.015, leg_h]] * 4,
                rgbaColors=[[0.85, 0.72, 0.5, 1]] + [[0.6, 0.6, 0.62, 1]] * 4,
                visualFramePositions=poss),
            [dk_x, dk_y, 0.0])
        p.changeDynamics(self.desk, -1, lateralFriction=0.20)  # メラミン天板の上で布は滑る

        # 雑巾の山（机の手前エッジから100mm）
        rag_x = dk_x + d / 2 - STACK_EDGE - P.RAG_X * MM / 2 + self.pert["stack_dx"]
        rag_y = dk_y + self.pert["stack_dy"]
        rag_q = _q(0, 0, math.radians(self.pert["stack_yaw"]))
        self.stack = []
        for i in range(STACK_N):
            r = self._box([P.RAG_X * MM / 2, P.RAG_Y * MM / 2, P.RAG_T * MM / 2],
                          [rag_x, rag_y, h + STACK_LIFT + P.RAG_T * MM * (i + 0.5)],
                          orn=rag_q, mass=P.RAG_MASS, rgba=(0.95, 0.78, 0.2, 1))
            p.changeDynamics(r, -1, lateralFriction=0.80, restitution=0.02)  # 布同士は噛む
            self.stack.append(r)
            self.stack_spawn.append((rag_x, rag_y))
        # 教壇（W600 × H200、Y方向に通し）
        self._box([P.PODIUM_H * MM * 1.5, 5.25, P.PODIUM_H * MM / 2],
                  [PODIUM_X, 0, P.PODIUM_H * MM / 2], rgba=(0.75, 0.6, 0.45, 1))
        # 旗（ポール + 横棒 3000mm）
        pole = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.012, height=FLAG_BAR_Z)
        pv = p.createVisualShape(p.GEOM_CYLINDER, radius=0.012, length=FLAG_BAR_Z,
                                 rgbaColor=(0.95, 0.95, 0.95, 1))
        p.createMultiBody(0, pole, pv, [FLAG_X, 0, FLAG_BAR_Z / 2])
        self.flag_bar = self._box([0.012, 0.30, 0.012], [FLAG_X, 0.31, FLAG_BAR_Z],
                                  rgba=(0.9, 0.2, 0.2, 1))
        self._box([0.004, 0.30, 0.9], [FLAG_X, 0.31, FLAG_BAR_Z - 0.9],
                  rgba=(0.9, 0.35, 0.35, 0.85))
        # 固定バケツ①②③
        for (bx, by, stand) in BUCKET_FIXED:
            if stand > 0:
                self._box([0.15, 0.15, stand / 2], [bx, by, stand / 2], rgba=(0.8, 0.75, 0.6, 1))
            bc = p.createCollisionShape(p.GEOM_CYLINDER, radius=P.BUCKET_DIA * MM / 2,
                                        height=P.BUCKET_H * MM)
            bv = p.createVisualShape(p.GEOM_CYLINDER, radius=P.BUCKET_DIA * MM / 2,
                                     length=P.BUCKET_H * MM, rgbaColor=(0.9, 0.9, 0.95, 0.6))
            p.createMultiBody(0, bc, bv, [bx, by, stand + P.BUCKET_H * MM / 2])

    # ------------------------------------------------------------------ robot
    def _load_robot(self):
        self.rid = p.loadURDF(URDF, [0, 0, 0.002], _q(0, 0, 0),
                              flags=p.URDF_USE_INERTIA_FROM_FILE)
        self.J = {}
        for i in range(p.getNumJoints(self.rid)):
            self.J[p.getJointInfo(self.rid, i)[1].decode()] = i
        # 車輪は接地支持のみ（横滑りさせる）
        for n in ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"):
            p.changeDynamics(self.rid, self.J[n], lateralFriction=0.35, spinningFriction=0.002,
                             rollingFriction=0.002)
        # 装填系・砲塔は位置制御
        for n, f in (("turret_yaw", 9.0), ("shooter_pitch", 28.0),
                     ("grabber_slide", 50.0), ("grabber_press", 50.0)):
            p.setJointMotorControl2(self.rid, self.J[n], p.POSITION_CONTROL,
                                    targetPosition=0.0, force=f)
        p.setJointMotorControl2(self.rid, self.J["shooter_pitch"], p.POSITION_CONTROL,
                                targetPosition=math.radians(P.PITCH_DEFAULT), force=28.0)
        for n in ("roller_upper", "roller_lower", "singulator"):
            p.setJointMotorControl2(self.rid, self.J[n], p.VELOCITY_CONTROL,
                                    targetVelocity=0.0, force=0.31)
        li = p.getDynamicsInfo(self.rid, -1)
        self.inertial_offset = (li[3], li[4])

    # base_link の設計座標系 <-> PyBullet の重心/主軸系 の変換
    def link_pose(self):
        pos, orn = p.getBasePositionAndOrientation(self.rid)
        inv = p.invertTransform(*self.inertial_offset)
        return p.multiplyTransforms(pos, orn, *inv)

    def nip_world(self):
        """射出点（ニップ）のワールド座標と射出方向。"""
        st = p.getLinkState(self.rid, self.J["shooter_pitch"], computeForwardKinematics=True)
        pos, orn = st[4], st[5]
        m = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        # 仰角リンクのローカル +X が射出方向
        return np.array(pos), m @ np.array([1.0, 0.0, 0.0])

    # -------------------------------------------------------------- controller
    def drive_to(self, target_x: float, yaw: float = 0.0):
        """走行系は「速度指令ドライブ」として扱う。

        メカナム＋PD力制御を PyBullet の摩擦モデルで安定させるのは本質的でないうえ、
        機構解析の対象は装填系・砲塔である。ここでは台形速度プロファイル（0.8m/s、
        1.5m/s²）をそのまま水平速度として与え、鉛直方向と姿勢は物理に任せる。
        その代わり「机に触れたら反則」を接触判定で厳密に検出する（self.fouls）。
        """
        (px, py, pz), orn = self.link_pose()
        # インターロック: グラバーが出ている間は走らせない（後方転倒・机接触）
        ok, why = self.interlocks.can_drive()
        if not ok and abs(target_x - px) > 0.01:
            if not self.violations or self.violations[-1][1] != why:
                self.violations.append((round(self.t_now, 2), why))
            target_x = px
        self.profile.cruise = self.interlocks.max_speed()
        self.cmd_x = self.profile.step(target_x, DT)
        vel, ang = p.getBaseVelocity(self.rid)
        vx = float(np.clip((self.cmd_x - px) / DT, -self.CRUISE * 1.6, self.CRUISE * 1.6))
        vy = float(np.clip(-py / DT, -0.4, 0.4))
        yaw_now = p.getEulerFromQuaternion(orn)[2]
        wz = float(np.clip((math.radians(yaw) - yaw_now) / DT, -2.0, 2.0))
        p.resetBaseVelocity(self.rid, [vx, vy, vel[2]], [ang[0], ang[1], wz])
        # 車輪速度はメカナム逆運動学から出す（control/tr_control.py と同じ実装）
        for n, w in zip(("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"),
                        self.mk.inverse(vx, vy, wz)):
            p.setJointMotorControl2(self.rid, self.J[n], p.VELOCITY_CONTROL,
                                    targetVelocity=w, force=3.0)

    def _grasp(self):
        """上押さえが降りた瞬間に、山を櫛歯へ拘束する（把持のモデル化）。

        「押さえと櫛歯で挟んだ布の束が一体で動く」ことを剛体で表現するための固定拘束。
        押さえを解放すると拘束も外れ、以降は傾斜した櫛歯の上を自重で滑ってホッパーへ落ちる。
        """
        link = self.J["fork_tilt"]
        st = p.getLinkState(self.rid, link, computeForwardKinematics=True)
        # createConstraint の parentFramePosition は「リンク重心フレーム」基準（PyBulletの仕様）
        inv = p.invertTransform(st[0], st[1])
        for i, r in enumerate(self.stack):
            pos, orn = p.getBasePositionAndOrientation(r)
            # 把持できる条件は2つ:
            #  (1) 櫛歯の矩形（長さ260 × 幅465）の上にあること
            #  (2) 挿入で 10mm 以上押されていないこと（押したなら「下に入れた」ではない）
            local = p.multiplyTransforms(inv[0], inv[1], pos, orn)[0]
            fw = ((P.FORK_TINES - 1) * P.FORK_PITCH + P.FORK_TINE_W) * MM / 2
            sx0, sy0 = self.stack_spawn[i]
            push = math.hypot(pos[0] - sx0, pos[1] - sy0)
            self.pushed_mm = max(self.pushed_mm, push * 1000.0)
            if not (-P.FORK_LEN * MM - 0.02 <= local[0] <= 0.02 and abs(local[1]) <= fw):
                continue
            if push > 0.010:
                continue
            # 櫛歯の上面に載せ直してから拘束する（天板を引きずらせない）
            fpos, _ = p.multiplyTransforms(st[4], st[5], [0, 0, 0], [0, 0, 0, 1])
            newpos = [pos[0], pos[1], fpos[2] + 0.002 + P.RAG_T * MM * (i + 0.5)]
            p.resetBasePositionAndOrientation(r, newpos, orn)
            rel_pos, rel_orn = p.multiplyTransforms(inv[0], inv[1], newpos, orn)
            c = p.createConstraint(self.rid, link, r, -1, p.JOINT_FIXED,
                                   [0, 0, 0], rel_pos, [0, 0, 0], rel_orn, [0, 0, 0, 1])
            p.changeConstraint(c, maxForce=60.0)
            self._grasp_cons.append(c)
        self.grasped = len(self._grasp_cons)

    def fire(self):
        """ニップ位置に雑巾を生成し、設計初速で射出する。"""
        pos, d = self.nip_world()
        pos = pos + d * 0.06
        r = self._box([P.RAG_X * MM / 2, P.RAG_Y * MM / 2, P.RAG_T * MM / 2],
                      pos.tolist(), mass=P.RAG_MASS, rgba=(0.2, 0.85, 0.95, 1))
        p.resetBaseVelocity(r, (d * MUZZLE_SPEED).tolist(), [0, -35.0, 0])  # バックスピン
        p.changeDynamics(r, -1, linearDamping=0.35, angularDamping=0.5,
                         lateralFriction=0.9, restitution=0.02)
        self.rags.append(r)
        return pos, d

    # -------------------------------------------------------------------- run
    def run(self, video: str | None):
        writer = None
        if video:
            os.makedirs(os.path.dirname(video), exist_ok=True)
            writer = subprocess.Popen(
                ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgba",
                 "-s", "960x720", "-r", str(FPS), "-i", "-", "-an",
                 "-vcodec", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", video],
                stdin=subprocess.PIPE)

        state = dict(base_x=0.0, yaw=0.0, pitch=P.PITCH_DEFAULT, slide=0.0, press=0.0,
                     roller=0.0, sing=0.0)
        phase_i, phase_name, fired = 0, MISSION[0][1], set()
        steps = int(T_END / DT)
        for k in range(steps):
            t = k * DT
            while phase_i + 1 < len(MISSION) and t >= MISSION[phase_i + 1][0]:
                phase_i += 1
                phase_name = MISSION[phase_i][1]
                tgt = MISSION[phase_i][2]
                for key, val in tgt.items():
                    if key == "fire":
                        continue
                    state[key] = val
                if "fire" in tgt and phase_i not in fired:
                    fired.add(phase_i)
                    self.fire()
                if MISSION[phase_i][2].get("slide") == 0.0 and phase_i >= 4:
                    state["sing"] = 25.0

            # --- 剛体では表現できない「くさびが布端を持ち上げる」挙動の代用 ---
            # 櫛歯挿入中だけ山の自重を打ち消し、櫛歯が山の下へ滑り込めるようにする。
            # 押さえが降りた時点で自重を戻し、以降は完全に物理任せ。
            if not self._clamped:
                if state["press"] > 0.04:
                    self._clamped = True
                    self._grasp()
                else:
                    for r in self.stack:
                        p.applyExternalForce(r, -1, [0, 0, P.RAG_MASS * 9.81],
                                             p.getBasePositionAndOrientation(r)[0], p.WORLD_FRAME)
            elif self._grasp_cons and state["press"] < 0.02:
                for c in self._grasp_cons:
                    p.removeConstraint(c)
                self._grasp_cons = []

            self.t_now = t
            # --- 挿入中の押し出し監視（実機のリミットSW相当） ---
            if not self._clamped and self.stack_spawn:
                push = 0.0
                for r, (sx0, sy0) in zip(self.stack, self.stack_spawn):
                    rp = p.getBasePositionAndOrientation(r)[0]
                    push = max(push, math.hypot(rp[0] - sx0, rp[1] - sy0))
                self.pushed_mm = max(self.pushed_mm, push * 1000.0)
                if push > ABORT_PUSH and not self.aborted:
                    self.aborted = True
                    self.abort_t = round(t, 2)
                    self.push_at_abort = push * 1000.0
            if self.aborted:
                # 中止: フォークを引き戻し、押さえは降ろさない（机を引きずらないため）
                state["slide"] = 0.0
                state["press"] = 0.0
            self.interlocks.grabber_extended = (
                p.getJointState(self.rid, self.J["grabber_slide"])[0] > 0.02)
            self.interlocks.moving = abs(p.getBaseVelocity(self.rid)[0][0]) > 0.02
            # クリープに落とすのは「机の手前200mm」だけ。接近全区間をクリープにすると
            # 3秒では届かず、フォーク挿入開始時にまだ走っていることになる。
            self.interlocks.near_desk = abs(self.cmd_x - APPROACH_X) < 0.20
            self.drive_to(state["base_x"] + self.pert["base_err"], state["yaw"])
            p.setJointMotorControl2(self.rid, self.J["turret_yaw"], p.POSITION_CONTROL,
                                    targetPosition=math.radians(state["yaw"]), force=9.0)
            p.setJointMotorControl2(self.rid, self.J["shooter_pitch"], p.POSITION_CONTROL,
                                    targetPosition=math.radians(state["pitch"]), force=28.0)
            p.setJointMotorControl2(self.rid, self.J["grabber_slide"], p.POSITION_CONTROL,
                                    targetPosition=state["slide"], force=50.0, maxVelocity=0.30)
            p.setJointMotorControl2(self.rid, self.J["grabber_press"], p.POSITION_CONTROL,
                                    targetPosition=state["press"], force=P.PRESS_FORCE_N, maxVelocity=0.10)
            for n, sgn in (("roller_upper", -1.0), ("roller_lower", 1.0)):
                p.setJointMotorControl2(self.rid, self.J[n], p.VELOCITY_CONTROL,
                                        targetVelocity=sgn * state["roller"], force=0.31)
            p.setJointMotorControl2(self.rid, self.J["singulator"], p.VELOCITY_CONTROL,
                                    targetVelocity=state["sing"], force=1.0)
            # 櫛歯傾斜は固定カム: 引込量の関数として一意に決まる受動関節
            s_now = p.getJointState(self.rid, self.J["grabber_slide"])[0]
            ratio = float(np.clip((CAM_START - s_now) / (CAM_START - CAM_END), 0.0, 1.0))
            p.setJointMotorControl2(self.rid, self.J["fork_tilt"], p.POSITION_CONTROL,
                                    targetPosition=math.radians(P.FORK_TILT_MAX) * ratio,
                                    force=30.0, maxVelocity=1.2)

            p.stepSimulation()
            for c in p.getContactPoints(bodyA=self.rid, bodyB=self.desk):
                if c[9] > 0.5:
                    self.fouls.append((round(t, 2), c[3], round(c[9], 1)))

            if k % SUBSTEPS == 0:
                self._record(t, phase_name)
                if writer:
                    writer.stdin.write(self._frame(t))

        if writer:
            writer.stdin.close()
            writer.wait()
        return self.log

    def _record(self, t, phase):
        (px, py, pz), orn = self.link_pose()
        js = p.getJointStates(self.rid, list(self.J.values()))
        names = list(self.J.keys())
        rec = dict(t=round(t, 3), phase=phase, base=[round(px, 4), round(py, 4), round(pz, 4)],
                   yaw_deg=round(math.degrees(p.getEulerFromQuaternion(orn)[2]), 2),
                   pitch_deg=round(math.degrees(p.getEulerFromQuaternion(orn)[1]), 2),
                   roll_deg=round(math.degrees(p.getEulerFromQuaternion(orn)[0]), 2))
        for n, s in zip(names, js):
            rec[f"q_{n}"] = round(s[0], 5)
            rec[f"tau_{n}"] = round(s[3], 4)
        dpos, _ = p.getBasePositionAndOrientation(self.desk)
        rec["desk_x"] = round(dpos[0], 5)
        rec["desk_y"] = round(dpos[1], 5)
        # フォーク先端（キャリッジリンク原点 + 設計オフセット）
        st = p.getLinkState(self.rid, self.J["grabber_slide"], computeForwardKinematics=True)
        rec["fork_tip"] = [round(v, 4) for v in st[4]]
        self.log.append(rec)

    def _frame(self, t: float = 0.0):
        (px, py, _), _ = self.link_pose()
        if self.cam == "close" and t < 12.5:
            # 装填のクローズアップ: 櫛歯の先端を追う
            st = p.getLinkState(self.rid, self.J["fork_tilt"], computeForwardKinematics=True)
            fx = st[4][0]
            vm = p.computeViewMatrix([fx + 0.55, -1.05, 1.16], [fx - 0.15, 0.02, 0.80], [0, 0, 1])
            pm = p.computeProjectionMatrixFOV(42, 960 / 720, 0.05, 30)
            _, _, rgb, _, _ = p.getCameraImage(960, 720, vm, pm, renderer=p.ER_TINY_RENDERER,
                                               flags=p.ER_NO_SEGMENTATION_MASK)
            return np.reshape(np.asarray(rgb, dtype=np.uint8), (720, 960, 4)).tobytes()
        vm = p.computeViewMatrix([px - 2.6, -2.9, 2.0], [px + 0.6, 0.0, 0.75], [0, 0, 1])
        pm = p.computeProjectionMatrixFOV(48, 960 / 720, 0.05, 30)
        _, _, rgb, _, _ = p.getCameraImage(960, 720, vm, pm, renderer=p.ER_TINY_RENDERER,
                                           flags=p.ER_NO_SEGMENTATION_MASK)
        return np.reshape(np.asarray(rgb, dtype=np.uint8), (720, 960, 4)).tobytes()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", default=None)
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--cam", default="wide", choices=["wide", "close"])
    ap.add_argument("--log", default=os.path.join(HERE, "..", "out", "sim", "mission_log.json"))
    args = ap.parse_args()
    video = None if args.no_video else (args.video or os.path.join(HERE, "..", "out", "sim",
                                                                   "tr_mission.mp4"))
    sim = Sim(cam=args.cam)
    log = sim.run(video)
    os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
    with open(args.log, "w") as f:
        json.dump(log, f, ensure_ascii=False)

    # サマリ
    last = log[-1]
    print(f"steps={len(log)}  最終フェーズ={last['phase']}")
    print(f"机の移動量: dx={last['desk_x'] - DESK_X:+.4f} m  dy={last['desk_y']:+.4f} m"
          "   ← 1mm でも動かすと反則 (4.1.4a)")
    if sim.fouls:
        t0 = sim.fouls[0]
        print(f"  ⚠ 机への接触 {len(sim.fouls)} 回  初回 t={t0[0]}s link={t0[1]} F={t0[2]}N")
    else:
        print("  机への接触: なし")
    # 回収成功数（ホッパー内寸に入った雑巾）
    (bx, by, _), _ = sim.link_pose()
    got = 0
    for r in sim.stack:
        rp = p.getBasePositionAndOrientation(r)[0]
        if (bx + P.HOP_X0 * MM - 0.02 <= rp[0] <= bx + P.HOP_X1 * MM + 0.02
                and abs(rp[1] - by) <= P.HOP_Y * MM + 0.02
                and rp[2] < (P.HOP_TOP_Z + 60) * MM):
            got += 1
    print(f"ホッパーに収まった雑巾: {got} / {STACK_N} 枚")
    if sim.aborted:
        print(f"挿入中止（リミットSW相当）: t={sim.abort_t}s  検知時の押し出し {sim.push_at_abort:.1f}mm / 最終 {sim.pushed_mm:.1f}mm")
    else:
        print(f"挿入中止: なし（山の押し出し最大 {sim.pushed_mm:.1f}mm）")
    if sim.violations:
        print(f"インターロック作動: {len(sim.violations)} 回")
        for t_, why in sim.violations[:3]:
            print(f"    t={t_}s  {why}")
    else:
        print("インターロック作動: なし（シーケンスが安全側で組めている）")
    for n in ("turret_yaw", "shooter_pitch", "grabber_slide", "grabber_press"):
        taus = [abs(r[f"tau_{n}"]) for r in log]
        print(f"  {n:14s} 最大トルク/推力 {max(taus):8.2f}")
    rolls = [abs(r["roll_deg"]) for r in log]
    pitches = [abs(r["pitch_deg"]) for r in log]
    print(f"車体姿勢の最大変化: roll {max(rolls):.2f}°  pitch {max(pitches):.2f}°")
    if video:
        print("video:", os.path.abspath(video))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
