// ─────────────────────────────────────────────────────────────────────────────
//  NaviCore cheat-sheet relay — a single Cloudflare Worker you (greghulette) host
//  ONCE for every user of config_tool. It lets the config tool's "📱 QR" button
//  upload a cheat-sheet HTML page directly from the browser (CORS-open) and hands
//  back a short, auto-expiring link the phone can scan. No accounts for anyone else.
//
//  ── One-time setup (Cloudflare dashboard, ~5 min, free tier) ──────────────────
//   1. Workers & Pages → Create → Create Worker. Name it e.g. "navicore-cheatsheet".
//      Deploy the starter, then Edit code → paste THIS file → Deploy.
//   2. Storage & Databases → KV → Create namespace, name it "CHEATSHEETS".
//   3. The Worker → Settings → Bindings → Add → KV namespace:
//         Variable name: CS      Namespace: CHEATSHEETS
//   4. (recommended) Settings → Variables and Secrets → add a Secret:
//         UPLOAD_TOKEN = <a long random string>
//      Put that SAME string in config_tool/index.html → CS_QR_TOKEN.
//   5. Copy the Worker URL (https://navicore-cheatsheet.<you>.workers.dev) into
//      config_tool/index.html → CS_QR_ENDPOINT.
//
//  Abuse guards (all here, tune the constants): 1 MB size cap, 1-day auto-delete,
//  a per-IP hourly rate limit, an optional shared token, and unguessable keys.
//  Free-tier headroom is huge (100k reads/day, 1k KV writes/day) — each upload is
//  ~2 writes, so ~500 uploads/day before you'd ever notice.
// ─────────────────────────────────────────────────────────────────────────────

const MAX_BYTES    = 1024 * 1024;    // reject anything over 1 MB (cheat sheets are ~tens of KB)
const TTL_SECONDS  = 24 * 60 * 60;   // links auto-delete after 1 day
const RATE_MAX     = 20;             // uploads allowed per IP per window
const RATE_WINDOW  = 60 * 60;        // rate-limit window: 1 hour
const KEY_LEN      = 64;             // max path length we'll look up

const CORS = {
  'Access-Control-Allow-Origin':  '*',
  'Access-Control-Allow-Methods': 'POST, GET, OPTIONS',
  'Access-Control-Allow-Headers': 'Content-Type, X-CS-Token',
  'Access-Control-Max-Age':       '86400',
};

const json = (obj, status = 200) =>
  new Response(JSON.stringify(obj), { status, headers: { 'Content-Type': 'application/json', ...CORS } });

// Served when a key isn't in KV yet. Right after an upload, KV can take a few seconds
// to propagate to the edge the phone hits — so instead of a hard 404, retry a few times,
// then show "expired". Keyed per-path so a genuinely-dead link gives up cleanly.
const RETRY_PAGE = `<!doctype html><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1"><title>Loading…</title><body style="font-family:-apple-system,Segoe UI,Roboto,sans-serif;text-align:center;padding:48px 20px;color:#333"><p id=m>Loading cheat sheet…</p><script>var k='csn'+location.pathname,n=+(sessionStorage[k]||0);if(n<8){sessionStorage[k]=n+1;setTimeout(function(){location.reload()},1500);}else{document.getElementById('m').textContent='This cheat-sheet link has expired or does not exist.';}</script></body>`;

export default {
  async fetch(request, env) {
    const url = new URL(request.url);

    if (request.method === 'OPTIONS') return new Response(null, { headers: CORS });

    // ── GET /<key> → serve the stored cheat sheet ──
    if (request.method === 'GET') {
      const key = url.pathname.replace(/^\/+/, '');
      if (!key) return new Response('NaviCore cheat-sheet relay.', { headers: { 'Content-Type': 'text/plain', ...CORS } });
      if (key.length > KEY_LEN || !/^[a-z0-9]+$/i.test(key))
        return new Response('Not found', { status: 404, headers: { 'Content-Type': 'text/plain', ...CORS } });
      const html = env.CS ? await env.CS.get('cs:' + key) : null;
      if (html != null) {
        const headers = { 'Content-Type': 'text/html; charset=utf-8', 'Cache-Control': 'no-store', ...CORS };
        // ?dl → serve as a real file download so a phone saves it to Files/Downloads
        // instead of just rendering it. The cheat sheet page's "📥 Save" button links
        // here; a bare GET (no ?dl) still renders inline for viewing/scanning.
        if (url.searchParams.has('dl'))
          headers['Content-Disposition'] = 'attachment; filename="NaviCore_CheatSheet.html"';
        return new Response(html, { headers });
      }
      return new Response(RETRY_PAGE, { headers: { 'Content-Type': 'text/html; charset=utf-8', 'Cache-Control': 'no-store', ...CORS } });
    }

    // ── POST / → store a new cheat sheet, return { link } ──
    if (request.method === 'POST') {
      if (!env.CS) return json({ error: 'KV binding "CS" is missing — see setup step 3' }, 500);

      if (env.UPLOAD_TOKEN && request.headers.get('X-CS-Token') !== env.UPLOAD_TOKEN)
        return json({ error: 'bad or missing token' }, 403);

      // Per-IP rate limit (approximate; KV isn't atomic, which is fine for this).
      const ip = request.headers.get('CF-Connecting-IP') || 'unknown';
      const rlKey = 'rl:' + ip;
      const count = parseInt((await env.CS.get(rlKey)) || '0', 10) || 0;
      if (count >= RATE_MAX) return json({ error: 'rate limit reached — try again later' }, 429);

      const cl = parseInt(request.headers.get('Content-Length') || '0', 10) || 0;
      if (cl > MAX_BYTES) return json({ error: 'too large' }, 413);
      const body = await request.text();
      if (!body) return json({ error: 'empty body' }, 400);
      if (body.length > MAX_BYTES) return json({ error: 'too large' }, 413);

      const key = crypto.randomUUID().replace(/-/g, '');
      await env.CS.put('cs:' + key, body, { expirationTtl: TTL_SECONDS });
      await env.CS.put(rlKey, String(count + 1), { expirationTtl: RATE_WINDOW });

      return json({ link: url.origin + '/' + key, expiresInSeconds: TTL_SECONDS });
    }

    return new Response('Method not allowed', { status: 405, headers: CORS });
  },
};
