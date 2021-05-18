var PreNeighbor = [];
function ReWordle() {
  var f;
  SavePreEditPosition();
  AddForce = !1;
  for (var e = 0, c = 0, a = (f = 1e3), b = 0; b < wordnum; b++) {
    var h = bodyList[b].GetWorldCenter();
    h.x > c && (c = h.x);
    h.y > e && (e = h.y);
    h.x < a && (a = h.x);
    h.y < f && (f = h.y);
  }
  h = { x: (c - a) / 2 + a, y: (e - f) / 2 + f };
  c = Math.min((c - a) / 2, (e - f) / 2);
  e = [];
  if (0 != multibodies.length) {
    multibodies.sort(sortNumber);
    for (b = 0; b < multibodies.length; b++) {
      c = multibodies[b];
      PreNeighbor[c] = wordsNeighbor[c];
      drawnBody.remove(bodyList[c]);
      for (a = bodyList[c].GetFixtureList(); a; a = a.m_next) a.SetSensor(!0);
      f = { id: c, r: 0 };
      e.push(f);
    }
    neighberMove(h, e, !0);
  } else {
    for (b = 0; b < wordnum; b++)
      if (
        ((a = bodyList[b].GetWorldCenter()),
        (f = a.x - h.x),
        (a = a.y - h.y),
        (f = Math.sqrt(f * f + a * a)),
        (PreNeighbor[b] = wordsNeighbor[b]),
        f > c * radiusRatio && 0.5 > we[b])
      ) {
        for (var a = !1, d = 0; d < constrainBody.length; d++)
          1 == constrainBody[d].indexOf(b) && (a = !0);
        if (!a) {
          drawnBody.remove(bodyList[b]);
          for (a = bodyList[b].GetFixtureList(); a; a = a.m_next)
            a.SetSensor(!0);
          f = { id: b, r: f };
          e.push(f);
        }
      }
    e.sort(sortNumber_forRewordle);
    neighberMove(h, e, Reflag);
  }
  iternum = 0;
  ifedit = !1;
  PreNeighbor = [];
}
function neighberMove(f, e, c) {
  if (c)
    for (c = 0; c < wordnum; c++) {
      e = !1;
      var a = bodyList[c].GetFixtureList();
      for (0.5 <= we[c] ? (a = a.m_next) : a; a; a = a.m_next)
        a.IsSensor() && (e = !0);
      if (e) {
        var b = bodyList[c].GetWorldCenter();
        var h = {
          x: bodyList[c].GetWorldCenter().x,
          y: bodyList[c].GetWorldCenter().y,
        };
        drawnBody.remove(bodyList[c]);
        var d = (b.x - f.x) * startPositionRatio + f.x;
        b = (b.y - f.y) * startPositionRatio + f.y;
        bodyList[c].SetPosition(new b2Vec2(d, b));
        b = placeBody(bodyList[c], !0);
        b = PickPosition(b, bodyList[c]);
        h = { id: c, startpos: h, endpos: b, count: 0, editflag: 3 };
        animation.push(h);
      }
    }
  else
    for (a = 0; a < e.length; a++) {
      c = e[a].id;
      b = bodyList[c].GetWorldCenter();
      h = {
        x: bodyList[c].GetWorldCenter().x,
        y: bodyList[c].GetWorldCenter().y,
      };
      drawnBody.remove(bodyList[c]);
      var g = Math.abs(b.x - f.x),
        n = Math.abs(b.y - f.y),
        l = 1e4,
        m = -1,
        k = [];
      for (d = a + 1; d < e.length; d++) k.push(e[d].id);
      k = RemoveSensor(PreNeighbor[c], k);
      void 0 == k[0] && (k = findNeighbor(!0, c));
      if (g > n)
        for (d = 0; d < k.length; d++)
          (g = bodyList[k[d]].GetWorldCenter()),
            0.5 <= we[k[d]] && (g = CorrectCenter(k[d], g)),
            (g = Math.abs(g.x - f.x)),
            l > g && ((l = g), (m = k[d]));
      else
        for (d = 0; d < k.length; d++)
          (g = bodyList[k[d]].GetWorldCenter()),
            0.5 <= we[k[d]] && (g = CorrectCenter(k[d], g)),
            (g = Math.abs(g.y - f.y)),
            l > g && ((l = g), (m = k[d]));
      d = m;
      k = bodyList[d].GetPosition();
      l = bodyList[d].GetWorldCenter();
      0.5 <= we[d] && (l = CorrectCenter(d, l));
      m = [
        {
          x: l.x * drawScale - bodywidth[d],
          y: l.y * drawScale - bodyheight[d],
        },
        {
          x: l.x * drawScale + bodywidth[d],
          y: l.y * drawScale - bodyheight[d],
        },
        {
          x: l.x * drawScale + bodywidth[d],
          y: l.y * drawScale + bodyheight[d],
        },
        {
          x: l.x * drawScale - bodywidth[d],
          y: l.y * drawScale + bodyheight[d],
        },
      ];
      g = bodyList[d].GetAngle();
      if (0 != g)
        for (d = 0; d < m.length; d++)
          (n =
            (m[d].x - k.x * drawScale) * Math.cos(g) -
            (m[d].y - k.y * drawScale) * Math.sin(g) +
            k.x * drawScale),
            (m[d].y =
              (m[d].x - k.x * drawScale) * Math.sin(g) +
              (m[d].y - k.y * drawScale) * Math.cos(g) +
              k.y * drawScale),
            (m[d].x = n);
      k = { x: -10, y: -10 };
      g = { x: b.x * drawScale, y: b.y * drawScale };
      n = { x: l.x * drawScale, y: l.y * drawScale };
      for (d = 0; 4 > d; d++) {
        var p = segmentsIntr(g, n, m[d], 3 == d ? m[0] : m[d + 1]);
        0 != p && (k = p);
      }
      -10 == k.x
        ? ((d = (b.x - l.x) * startPositionRatio + l.x),
          (b = (b.y - l.y) * startPositionRatio + l.y),
          bodyList[c].SetPosition(new b2Vec2(d, b)))
        : bodyList[c].SetPosition(new b2Vec2(k.x / drawScale, k.y / drawScale));
      b = placeBody(bodyList[c], !0);
      b = PickPosition(b, bodyList[c]);
      h = { id: c, startpos: h, endpos: b, count: 0, editflag: 3 };
      animation.push(h);
    }
}
function PickPosition(f, e) {
  for (var c, a, b = [], h = [], d = 0; d < wordnum; d++)
    for (a = bodyList[d].GetFixtureList(); a; a = a.m_next)
      a.IsSensor() && h.push(d);
  h = RemoveSensor(PreNeighbor[e.GetID()], h);
  a = [];
  for (d = 0; d < h.length; d++) a.push(wo[h[d]]);
  for (d = 0; d < f.length; d++) {
    a = f[d].x;
    c = f[d].y;
    e.SetPosition(new b2Vec2(a, c));
    a = findNeighbor(!0, e.GetID());
    c = [];
    for (var g = 0; g < a.length; g++) c.push(wo[a[g]]);
    b[d] = EvaluateScore(a, h);
  }
  b = b.indexOf(Math.max.apply(Math, b));
  a = f[b].x;
  c = f[b].y;
  e.SetPosition(new b2Vec2(a, c));
  drawnBody.push(e);
  return { x: a, y: c };
}
function RemoveSensor(f, e) {
  for (var c = [], a = 0; a < f.length; a++) {
    for (var b = !1, h = 0; h < e.length; h++) f[a] == e[h] && (b = !0);
    b || c.push(f[a]);
  }
  return c;
}
function EvaluateScore(f, e) {
  for (var c = 0, a = 0, b = 0; a < f.length && b < e.length; )
    f[a] != e[b] ? (f[a] < e[b] ? ++a : ++b) : (c++, ++a, ++b);
  return c;
}
function shouldReWordle() {
  for (var f = 0, e = 0, c = 1e3, a = 1e3, b = 0; b < wordnum; b++) {
    var h = bodyList[b].GetWorldCenter();
    0.5 <= we[b] && (h = CorrectCenter(b, h));
    h.x > e && (e = h.x);
    h.y > f && (f = h.y);
    h.x < a && (a = h.x);
    h.y < c && (c = h.y);
  }
  for (
    var h = (e - a) / 2 + a,
      d = (f - c) / 2 + c,
      f = Math.min((e - a) / 2, (f - c) / 2),
      b = 0;
    b < wordnum;
    b++
  )
    if (
      ifattract[b] &&
      ((e = (10 * Math.PI) / 180),
      (c = bodyList[b].GetAngle()),
      (c < e && c > -e) ||
        (c > Math.PI / 2 - e && c < Math.PI / 2 + e) ||
        (c > -Math.PI / 2 - e && c < -Math.PI / 2 + e))
    ) {
      a = bodyList[b].GetWorldCenter();
      0.5 <= we[b] && (a = CorrectCenter(b, a));
      var g = a.x - h,
        n = a.y - d;
      if (Math.sqrt(g * g + n * n) <= 0.8 * f)
        for (g = 0; g < wordsNeighbor[b].length; g++) {
          var n = wordsNeighbor[b][g],
            l = bodyList[n].GetWorldCenter();
          0.5 <= we[n] && (l = CorrectCenter(n, l));
          if (ifattract[n]) {
            var m = bodyList[n].GetAngle();
            if (
              (m < e && m > -e) ||
              (m > Math.PI / 2 - e && m < Math.PI / 2 + e) ||
              (m > -Math.PI / 2 - e && m < -Math.PI / 2 + e)
            ) {
              var k = bodywidth[b],
                p = bodyheight[b],
                q = bodywidth[n],
                r = bodyheight[n];
              if (
                (c > Math.PI / 2 - e && c < Math.PI / 2 + e) ||
                (c > -Math.PI / 2 - e && c < -Math.PI / 2 + e)
              )
                (k = bodyheight[b]), (p = bodywidth[b]);
              if (
                (m > Math.PI / 2 - e && m < Math.PI / 2 + e) ||
                (m > -Math.PI / 2 - e && m < -Math.PI / 2 + e)
              )
                (q = bodyheight[n]), (r = bodywidth[n]);
              m = Math.abs(l.x - a.x) * drawScale - k - q;
              if (40 < Math.abs(l.y - a.y) * drawScale - p - r) {
                if (IfAnotherOK(!1, n, b))
                  return (
                    (preReWordleReason[0] = n), (preReWordleReason[1] = b), !0
                  );
              } else if (70 < m && IfAnotherOK(!0, n, b))
                return (
                  (preReWordleReason[0] = n), (preReWordleReason[1] = b), !0
                );
            }
          }
        }
    }
  return !1;
}
function IfAnotherOK(f, e, c) {
  var a = bodyList[c].GetWorldCenter();
  0.5 <= we[c] && (a = CorrectCenter(c, a));
  var b = bodyList[e].GetWorldCenter();
  0.5 <= we[e] && (b = CorrectCenter(e, b));
  var h = a.x,
    d = b.x,
    a = a.y,
    g = b.y;
  h > d ? ((b = h), (h = d)) : (b = d);
  a > g ? ((d = a), (a = g)) : (d = g);
  if (f) {
    for (var n = 0, l = -10, m = 1e3, k = -10, g = 0; g < wordnum; g++)
      if (g != c && g != e) {
        var p = bodyList[g].GetWorldCenter();
        0.5 <= we[g] && (p = CorrectCenter(g, p));
        f = p.y;
        p = p.x;
        p > h &&
          p < b &&
          (f > d && f < m && ((m = f), (k = g)),
          f < a && f > n && ((n = f), (l = g)));
      }
    if (-10 != l && -10 != k)
      return (
        (e = bodyheight[l]),
        (c = bodyheight[k]),
        0 != bodyList[l].GetAngle() && (e = bodywidth[l]),
        0 != bodyList[k].GetAngle() && (c = bodywidth[k]),
        35 < Math.abs(n - m) * drawScale - e - c ? !0 : !1
      );
  } else {
    n = 0;
    l = -10;
    m = 1e3;
    k = -10;
    for (g = 0; g < wordnum; g++)
      g != c &&
        g != e &&
        ((p = bodyList[g].GetWorldCenter()),
        0.5 <= we[g] && (p = CorrectCenter(g, p)),
        (f = p.y),
        (p = p.x),
        f > a &&
          f < d &&
          (p > b && p < m && ((m = p), (k = g)),
          p < h && p > n && ((n = p), (l = g))));
    if (-10 != l && -10 != k)
      return (
        (e = bodywidth[l]),
        (c = bodywidth[k]),
        0 != bodyList[l].GetAngle() && (e = bodyheight[l]),
        0 != bodyList[k].GetAngle() && (c = bodyheight[k]),
        45 < Math.abs(n - m) * drawScale - e - c ? !0 : !1
      );
  }
  return !0;
}
function sortNumber_forRewordle(f, e) {
  return f.r + 1 / we[f.id] / 5 - (e.r + 1 / we[e.id] / 5);
}
