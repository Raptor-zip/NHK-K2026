// Cloudflare Pages アドバンストモードのワーカ。
// 目的: Google Analytics をファーストパーティ配信する。ブラウザ拡張・トラッキング保護・DNSフィルタは
// googletagmanager.com / google-analytics.com 宛てを遮断するが、自ドメイン(/ga/*)経由なら回避できる。
// /ga/* を Google へ中継し、それ以外のパスは通常どおり静的アセット(env.ASSETS)を返す。

const GTM = 'https://www.googletagmanager.com';
const GA = 'https://www.google-analytics.com';

export default {
  async fetch(request, env) {
    const url = new URL(request.url);
    const path = url.pathname;

    if (path.startsWith('/ga/')) {
      const rest = path.slice(3); // '/ga' を除いた残り。例: /gtag/js, /g/collect
      // スクリプト系は googletagmanager、計測ヒットは google-analytics へ振り分ける
      const toGtm = rest.startsWith('/gtag/') || rest.startsWith('/gtm.js') || rest.startsWith('/gtm/');
      const target = (toGtm ? GTM : GA) + rest + url.search;

      const headers = new Headers();
      // 訪問者の実IPを渡し、GA側のジオ集計を保つ
      const ip = request.headers.get('CF-Connecting-IP');
      if (ip) headers.set('X-Forwarded-For', ip);
      // User-Agent と Client Hints(Sec-CH-UA-*) を転送 → GA4 が端末/ブラウザ/OSを判定できる。
      // (UA文字列だけでも device_category/ブラウザ/OS は判定されるが、Chrome の UA縮約に備え
      //  Sec-CH-UA 系ヘッダも渡して精度を上げる。)
      for (const [k, v] of request.headers) {
        const lk = k.toLowerCase();
        if (
          lk === 'user-agent' ||
          lk === 'accept-language' ||
          lk === 'content-type' ||
          lk === 'referer' ||
          lk.startsWith('sec-ch-ua')
        ) {
          headers.set(k, v);
        }
      }

      const method = request.method;
      const body = method === 'GET' || method === 'HEAD' ? undefined : await request.arrayBuffer();
      let upstream;
      try {
        upstream = await fetch(target, { method, headers, body });
      } catch {
        return new Response('', { status: 502 });
      }

      const out = new Response(upstream.body, {
        status: upstream.status,
        statusText: upstream.statusText,
      });
      const ct = upstream.headers.get('content-type');
      if (ct) out.headers.set('content-type', ct);
      out.headers.set('cache-control', 'no-store');
      out.headers.set('x-ga-proxy', '1'); // 疎通確認用
      return out;
    }

    // それ以外は静的アセット(SPA)を配信。HTMLレスポンスからは Cloudflare Web Analytics の
    // 自動注入ビーコン(static.cloudflareinsights.com / data-cf-beacon)を剥がす。
    const resp = await env.ASSETS.fetch(request);
    const ct = resp.headers.get('content-type') || '';
    if (ct.includes('text/html')) {
      return new HTMLRewriter()
        .on('script[src*="cloudflareinsights"]', { element: (el) => el.remove() })
        .on('script[data-cf-beacon]', { element: (el) => el.remove() })
        .transform(resp);
    }
    return resp;
  },
};
