import { writable } from 'svelte/store';
import type { HudSnapshot, PerfSnapshot } from '../render/three-scene';
import type { TeamId } from '../config/field';
import type { ControlMode } from '../sim/types';
import { clonedParams, type SimParams, type StrategyVariant } from '../config/params';

export const hud = writable<HudSnapshot | null>(null);
export const perf = writable<PerfSnapshot | null>(null);

export const teamNames = writable<{ blue: string; red: string }>({
  blue: '青チーム',
  red: '赤チーム',
});

export interface UiPrefs {
  playing: boolean;
  speed: number;
  variant: StrategyVariant;
  redVariant: StrategyVariant;
  params: SimParams;
  oppSkill: number;
  blueCycleScale: number;
  redCycleScale: number;
  blueProbMul: number;
  redProbMul: number;
  blueDriveMul: number;
  redDriveMul: number;
  blueMuzzleY: number;
  redMuzzleY: number;
  blueSwerve: boolean;
  redSwerve: boolean;
  playerTeam: TeamId;
  showLidar: boolean;
  showCam: boolean;
  showPath: boolean;
  sound: boolean;
  controlMode: ControlMode;
  showPad: boolean;
  faceCam: boolean;
}

export const prefs = writable<UiPrefs>({
  playing: false,
  speed: 1,
  variant: 'optimal',
  redVariant: 'bucketFocus',
  params: clonedParams(),
  oppSkill: 0.7,
  blueCycleScale: 1,
  redCycleScale: 1.15,
  blueProbMul: 1,
  redProbMul: 1,
  blueDriveMul: 1,
  redDriveMul: 1,
  blueMuzzleY: 1.5,
  redMuzzleY: 1.2,
  blueSwerve: false,
  redSwerve: true,
  playerTeam: 'blue',
  showLidar: true,
  showCam: true,
  showPath: true,
  sound: true,
  controlMode: 'auto',
  showPad: true,
  faceCam: false,
});
