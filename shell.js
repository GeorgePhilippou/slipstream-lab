// Slipstream shell: module routing, inspector mounting, project switching, top-bar condition read-out.
(() => {
const { $, fmt, int } = SL;
let cur = null, refresh = null;
function cond() {
  const P = SL.P, F = P.flight, atm = SL.isa(F.h);
  $('cond').innerHTML = `m <b>${int(P.mass)} kg</b> · h <b>${int(F.h)} m</b> · V <b>${fmt(F.V, 0)} m/s</b> (${Math.round(F.V * 1.944)} kt) · M <b>${fmt(F.V / atm.a, 2)}</b> · OAT <b>${fmt(atm.Tc, 1)} °C</b>`;
  if (cur) $('crumb').innerHTML = `${P.name} / <b>${SL.modules[cur].title}</b>`;
}
function mount(key) {
  if (cur) { SL.modules[cur].deactivate && SL.modules[cur].deactivate(); $('cv-' + cur).hidden = true; }
  cur = key;
  const m = SL.modules[key];
  document.querySelectorAll('#mods a').forEach(a => a.setAttribute('aria-current', a.getAttribute('href') === '#' + key ? 'page' : 'false'));
  $('vp-bar').innerHTML = ''; if (m.toolbar) m.toolbar($('vp-bar'));
  refresh = SL.inspector($('inspector'), m.inspector());
  $('vp-hud').hidden = true; $('vp-legend').hidden = true; $('vp-empty').hidden = true; $('vp-toast').hidden = true;
  $('results').innerHTML = ''; $('res-note').textContent = '';
  $('cv-' + key).hidden = false;
  cond();
  m.activate && m.activate();
}
SL.refreshInspector = () => refresh && refresh();
SL.remountInspector = () => { if (cur) refresh = SL.inspector($('inspector'), SL.modules[cur].inspector()); };
SL.current = () => cur;
function route() { let h = location.hash.slice(1); if (!SL.modules[h]) h = 'geometry'; if (h !== cur) mount(h); }
addEventListener('hashchange', route);
$('project').onchange = e => SL.loadProject(e.target.value);
SL.on('project', d => {
  cond();
  if (d.full) SL.remountInspector(); else SL.refreshInspector();
  const m = SL.modules[cur]; if (m && m.onProject) m.onProject(d);
});
cond(); route();
})();
