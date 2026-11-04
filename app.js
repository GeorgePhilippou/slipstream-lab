// View router: #wing, #aircraft, #tunnel3d
(() => {
const names = ['wing', 'designer', 'aircraft', 'tunnel3d'];
let cur = null;
function show() {
  let h = location.hash.slice(1);
  if (!names.includes(h)) h = 'wing';
  if (cur === h) return;
  if (cur) SL.views[cur].deactivate();
  for (const n of names) {
    SL.$('v-' + n).hidden = n !== h;
    document.querySelector(`.tabs a[href="#${n}"]`).setAttribute('aria-selected', String(n === h));
  }
  cur = h;
  SL.views[h].activate();
}
addEventListener('hashchange', show);
show();
})();
