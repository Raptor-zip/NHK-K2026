// GA4 計測。標準 gtag.js はこの環境(拡張/トラッキング保護)では読み込めても一切送信しない
// (dataLayer は処理するが client_id もクッキーも作らず collect ゼロ) ことを実測で確認したため、
// gtag に依存せず GA4 の計測エンドポイント(/g/collect)へ直接ヒットを送る。本番は自ドメイン(/ga)
// 経由のファーストパーティ配信でブロックを回避する(GA4 到達を検証済み)。画面サイズ・ブラウザ・OS・
// デバイス等は自前でパラメータに載せる(gtag が自動で載せる分を代替する)。

type AnalyticsConsent = 'granted' | 'denied';

const env = import.meta.env as Record<string, string | undefined>;
const GA_ID = env.VITE_GA_MEASUREMENT_ID?.trim();
const CONSENT_KEY = 'k2026.gaConsent';
const CID_KEY = 'k2026.gaCid';
const SESS_KEY = 'k2026.gaSess';

// 本番は自ドメイン中継(/ga/g/collect → google-analytics.com)、dev は直 google。
const COLLECT = import.meta.env.PROD
  ? `${location.origin}/ga/g/collect`
  : 'https://www.google-analytics.com/g/collect';

function storage(): Storage | null {
  try {
    return window.localStorage;
  } catch {
    return null;
  }
}

export function analyticsAvailable(): boolean {
  return Boolean(GA_ID);
}

export function getAnalyticsConsent(): AnalyticsConsent | null {
  const value = storage()?.getItem(CONSENT_KEY);
  return value === 'granted' || value === 'denied' ? value : null;
}

/** 永続クライアントID (GA4 形式) とセッション情報。無ければ生成し、初回訪問/新セッションを判定する。 */
function identity(): {
  cid: string;
  sid: string;
  sct: number;
  newSession: boolean;
  firstVisit: boolean;
} {
  const s = storage();
  const nowMs = Date.now();
  const nowSec = Math.floor(nowMs / 1000);

  let cid = s?.getItem(CID_KEY) ?? '';
  const firstVisit = !cid;
  if (!cid) {
    cid = `${Math.floor(Math.random() * 1e10)}.${nowSec}`;
    s?.setItem(CID_KEY, cid);
  }

  // 30 分無操作でセッションリセット。GA4 のセッション/ユーザー集計用に sid/sct を持つ。
  let sid = nowSec;
  let sct = 1;
  let newSession = true;
  const raw = s?.getItem(SESS_KEY);
  if (raw) {
    const [pSid, pLast, pCnt] = raw.split('.').map(Number);
    if (pSid && pLast && nowMs - pLast < 30 * 60 * 1000) {
      sid = pSid;
      sct = pCnt || 1;
      newSession = false;
    } else {
      sct = (pCnt || 1) + 1;
    }
  }
  s?.setItem(SESS_KEY, `${sid}.${nowMs}.${sct}`);
  return { cid, sid: String(sid), sct, newSession, firstVisit };
}

/** 高エントロピー UA クライアントヒントを収集し、GA4 の環境詳細(ブラウザ/OS/デバイス)を精緻化する。 */
async function clientHints(): Promise<Record<string, string>> {
  const out: Record<string, string> = {};
  const uad = (
    navigator as Navigator & {
      userAgentData?: {
        platform?: string;
        mobile?: boolean;
        getHighEntropyValues?: (h: string[]) => Promise<Record<string, unknown>>;
      };
    }
  ).userAgentData;
  if (!uad) return out; // Firefox/Safari 等 → UA文字列(ワーカが転送)で最低限判別される
  if (uad.platform) out['uap'] = uad.platform;
  out['uamb'] = uad.mobile ? '1' : '0';
  try {
    const h = await uad.getHighEntropyValues?.([
      'architecture',
      'bitness',
      'model',
      'platformVersion',
      'fullVersionList',
      'wow64',
    ]);
    if (h) {
      if (h['architecture']) out['uaa'] = String(h['architecture']);
      if (h['bitness']) out['uab'] = String(h['bitness']);
      if (h['model']) out['uam'] = String(h['model']);
      if (h['platformVersion']) out['uapv'] = String(h['platformVersion']);
      if (h['wow64']) out['uaw'] = '1';
      const fvl = h['fullVersionList'] as Array<{ brand: string; version: string }> | undefined;
      if (Array.isArray(fvl)) out['uafvl'] = fvl.map((b) => `${b.brand};${b.version}`).join('|');
    }
  } catch {
    /* 取得不可でもUA文字列で最低限は判別される */
  }
  return out;
}

/** 1ヒットを GA4 /g/collect (本番は自ドメイン中継) へ送る。同意済みのときだけ。 */
function collect(en: string, extra?: Record<string, string>): void {
  if (!GA_ID || getAnalyticsConsent() !== 'granted') return;
  const id = identity();
  const p = new URLSearchParams({
    v: '2',
    tid: GA_ID,
    gtm: '45je',
    _p: String(Date.now()),
    cid: id.cid,
    sid: id.sid,
    sct: String(id.sct),
    seg: '1',
    en,
    _s: '1',
  });
  if (en === 'page_view') {
    if (id.newSession) p.set('_ss', '1'); // セッション開始
    if (id.firstVisit) p.set('_fv', '1'); // 初回訪問
  }
  if (extra) for (const [k, v] of Object.entries(extra)) p.set(k, v);
  const url = `${COLLECT}?${p.toString()}`;
  try {
    if (navigator.sendBeacon && navigator.sendBeacon(url)) return;
  } catch {
    /* fallthrough */
  }
  fetch(url, { method: 'POST', keepalive: true, mode: 'no-cors' }).catch(() => {});
}

let pageSent = false;

/** page_view を送る。1読み込み1回。画面サイズ・言語・UAヒント(ブラウザ/OS/デバイス)を載せる。 */
async function sendPageView(): Promise<void> {
  if (pageSent) return;
  pageSent = true;
  const hints = await clientHints();
  collect('page_view', {
    dl: location.href,
    dr: document.referrer,
    dt: document.title,
    sr: `${screen.width}x${screen.height}`,
    vp: `${Math.round(window.innerWidth)}x${Math.round(window.innerHeight)}`,
    ul: navigator.language || '',
    ...hints,
  });
}

/** ページ読込時。既に同意済みなら page_view を送る。未選択なら同意バナー(App側)の選択を待つ。 */
export function initAnalytics(): void {
  void sendPageView();
}

export function setAnalyticsConsent(consent: AnalyticsConsent): void {
  storage()?.setItem(CONSENT_KEY, consent);
  if (consent === 'granted') void sendPageView();
}

/**
 * カスタムイベント (match_start, param_change 等)。数値パラメータは epn.、文字列は ep. で送る。
 * GA4 の「イベント」レポートで件数、登録したカスタムディメンション/指標で内訳を集計できる。
 */
export function trackEvent(name: string, params?: Record<string, string | number | boolean>): void {
  const extra: Record<string, string> = {};
  if (params) {
    for (const [k, v] of Object.entries(params)) {
      if (typeof v === 'number' && Number.isFinite(v)) extra[`epn.${k}`] = String(v);
      else extra[`ep.${k}`] = String(v);
    }
  }
  collect(name, extra);
}
