var world = void 0,
  wo,
  we,
  fixDef = new b2FixtureDef(),
  bodyDef = new b2BodyDef(),
  drawScale = 30,
  canvaswidth = 1300,
  canvasheight = 600,
  backgroundcolor = "#FFFFFF",
  defaultfont = "HoboStd",
  colorthemenum = 0,
  bodyList = [],
  ifsubdivided = [],
  ifattract = [],
  tagColor = [],
  tagFont = [],
  drawnBody = [],
  bodywidth = [],
  bodyheight = [],
  bodydesent = [],
  prePosition = [],
  preEditInfo = [],
  preEditPoint = 0,
  preMovePosition = [],
  wordsNeighbor = [],
  preReWordleReason = [-1, -1],
  constrainBody = [],
  fw = [0, 0, 0, 0, 0, Math.PI / 2, -Math.PI / 2],
  color = [
    "#1a984f #67bd64 #a7d96a #fdae61 #f56d43 #fe5840".split(" "),
    "#c98b2f #803c27 #c56520 #e1b41b #807916 #e4de26".split(" "),
    "#ffec94 #ffaeae #fff0aa #b0e57c #b4d8e7 #56baec".split(" "),
    "#504b2e #a8be40 #8bc27e #c5ddd0 #c1d63d #7faf35".split(" "),
    "#1f0207 #52030d #95070e #a3230d #bd5600 #e4de26".split(" "),
    "#084233 #4b9100 #c7bd00 #ffd83d #b22400 #d58d2a".split(" "),
    "#468966 #fff0a5 #ffb03b #b64926 #8e2800 #294b81".split(" "),
    "#2b3b00 #636e00 #f7b100 #eb4200 #6b0019 #74839b".split(" "),
    "#064210 #2e5606 #a69c10 #746010 #422e10 #7cc572".split(" "),
    "#1f497d #4f81bd #c0504d #9bbd59 #8064a2 #4bacc6".split(" "),
  ],
  planet = [],
  multibodies = [],
  pushingbar = void 0,
  iternum = 9e3,
  wordnum,
  havedecent = !1,
  isFirstContrust = !0,
  isFirstSystemstart = !0,
  ifstart = !1,
  ifedit = !1,
  ifaddconstrain = !1,
  isshift = !1,
  isctrl = !1,
  isz = !1,
  ismousemulti = !1,
  animation = [],
  context,
  context2 = document.getElementById("canvas2").getContext("2d"),
  radiusRatio = 0.8,
  startPositionRatio = 0.5,
  Reflag = !0,
  Ifdebug = !1,
  usecenter = !0,
  ifAllFixed = !0,
  maxfontsize = 65,
  minfontsize = 2,
  arapweight = 13,
  centerweight = 0.7,
  AddForce = !0,
  UserReWordle = !1,
  UserAddForce = !0,
  seebox = !1,
  hullPoints = void 0;
function init(a, b) {
  canvasPosition = getElementPosition();
  isFirstSystemstart && (createWorld(), createWall(), debugDraw());
  createObject(a, b);
  isFirstSystemstart &&
    (window.setInterval(update, 1e3 / 60), (isFirstSystemstart = !1));
}
function createWorld() {
  void 0 == world &&
    ((world = new b2World(new b2Vec2(0, 0), !0)), world.SetWarmStarting(!1));
}
function createWall() {
  fixDef.density = 1;
  fixDef.friction = 0.5;
  fixDef.restitution = 0.2;
  bodyDef.type = b2Body.b2_staticBody;
  fixDef.shape = new b2PolygonShape();
  fixDef.shape.SetAsBox(canvaswidth / 30, 0.3);
  bodyDef.position.Set(canvaswidth / 30 / 2, canvasheight / 30 + 0.2);
  world.CreateBody(bodyDef).CreateFixture(fixDef);
  bodyDef.position.Set(canvaswidth / 30 / 2, -0.2);
  world.CreateBody(bodyDef).CreateFixture(fixDef);
  fixDef.shape.SetAsBox(0.3, canvasheight / 30);
  bodyDef.position.Set(-0.2, canvasheight / 30);
  world.CreateBody(bodyDef).CreateFixture(fixDef);
  bodyDef.position.Set(canvaswidth / 30 + 0.2, canvasheight / 30);
  world.CreateBody(bodyDef).CreateFixture(fixDef);
  bodyDef.type = b2Body.b2_staticBody;
  fixDef.shape = new b2CircleShape(0.01 / drawScale);
  bodyDef.position.Set(
    canvaswidth / drawScale / 2,
    canvasheight / drawScale / 2
  );
  var a = world.CreateBody(bodyDef);
  fixDef.isSensor = !0;
  a.CreateFixture(fixDef);
  planet.push(a);
}
function createObject(a, b) {
  for (var c = 0; c < wordnum; c++) {
    var d = createOffSet();
    var e = createOffSet();
    d = canvaswidth / 2 + 3.5 * d;
    e = canvasheight / 2 + 0.7 * e;
    void 0 == a || void 0 == b
      ? createBox(d, e, we[c], c, !1)
      : createBox(a[c], b[c], we[c], c, !0);
  }
}
function createBox(a, b, c, d, e) {
  tagFont[d] = defaultfont;
  fixDef.isSensor = !1;
  fixDef.density = 1;
  fixDef.friction = 0.2;
  fixDef.shape = new b2PolygonShape();
  var f = (maxfontsize - minfontsize) * c + minfontsize;
  context2.font = f + "px " + tagFont[d];
  var g = context2.measureText(wo[d]).width;
  bodywidth[d] = g / 2;
  bodyheight[d] = f / 2;
  bodydesent[d] = 0;
  ifsubdivided[d] = !1;
  bodyDef.type = b2Body.b2_dynamicBody;
  e || bodyDef.position.Set(a / drawScale, b / drawScale);
  var h = world.CreateBody(bodyDef);
  CreateLittleBody(g, f, h, d);
  e &&
    ((b = {
      x: (a + bodywidth[d] + canvaswidth / 2) / drawScale,
      y:
        (b -
          bodyheight[d] -
          (f / 2 - bodyheight[d] + bodydesent[d]) +
          canvasheight / 2) /
        drawScale,
    }),
    (a = { x: 0, y: 0 }),
    (a.y =
      300 >= 30 * b.y
        ? (250 + 2 * createOffSet()) / drawScale
        : (350 + 2 * createOffSet()) / drawScale),
    (a.x =
      650 >= 30 * b.x
        ? (550 + 2 * createOffSet()) / drawScale
        : (750 + 2 * createOffSet()) / drawScale),
    h.SetPosition(new b2Vec2(a.x, a.y)),
    h.m_force.SetZero(),
    (h.m_torque = 0),
    h.SetAngularVelocity(0),
    h.SetLinearVelocity(new b2Vec2(0, 0)),
    (a = { id: d, startpos: a, endpos: b, count: 0, editflag: 0 }),
    animation.push(a),
    drawnBody.push(h));
  h.SetID(d);
  bodyList[d] = h;
  tagColor[d] = createColor();
  0.7 >= c && h.SetAngle(createAngle());
  ifattract[d] = !0;
  h.SetFixedRotation(ifAllFixed);
  if (h.IsFixedRotation() && !e) {
    for (c = h.GetFixtureList(); c; c = c.m_next) c.SetSensor(!0);
    b = placeBody(h, !1);
    void 0 != prePosition[d]
      ? (a = { x: prePosition[d].x, y: prePosition[d].y })
      : ((a = { x: 0, y: 0 }),
        (a.y =
          300 >= 30 * b.y
            ? (250 + 2 * createOffSet()) / drawScale
            : (350 + 2 * createOffSet()) / drawScale),
        (a.x =
          650 >= 30 * b.x
            ? (550 + 2 * createOffSet()) / drawScale
            : (750 + 2 * createOffSet()) / drawScale));
    a = { id: d, startpos: a, endpos: b, count: 0, editflag: 0 };
    animation.push(a);
  }
}
function CreateLittleBody(a, b, c, d) {
  if (0.5 <= we[d]) {
    var e = -10;
    var f = -10;
    var g = -(a / 2),
      h = new b2FixtureDef();
    h.isSensor = !1;
    h.friction = 0.2;
    h.shape = new b2PolygonShape();
    for (var k = 0; k < wo[d].length; k++) {
      var l = measureTextH_W(context2, 0, 0, 250, 300, b, tagFont[d], wo[d][k]);
      var n = l.width / 2;
      var p = l.height / 2;
      var m = l.desent + (b / 2 - p);
      var q = g + n;
      m = [
        [-n + q, -p + m],
        [n + q, -p + m],
        [n + q, p + m],
        [-n + q, p + m],
      ];
      h.shape.SetAsArray(createVerticesArray(m), 4);
      c.CreateFixture(h);
      g += 2 * n;
      e < p && (e = p);
      f < l.desent && (f = l.desent);
    }
    l = measureTextH_W(context2, 0, 0, 250, 300, b, tagFont[d], "e");
    p = l.height / 2;
    n = a / 2;
    measureTextH_W(context2, 0, 0, 250, 300, b, tagFont[d], "l");
    m = b / 2 - p;
    m = [
      [-n, -p + m],
      [n, -p + m],
      [n, p + m],
      [-n, p + m],
    ];
    h.shape.SetAsArray(createVerticesArray(m), 4);
    c.CreateFixture(h);
    bodyheight[d] = (2 * e + f) / 2;
    bodydesent[d] = f;
    0 < f && (havedecent = !0);
    ifsubdivided[d] = !0;
  } else {
    f = -10;
    for (k = 0; k < wo[d].length; k++)
      (l = measureTextH_W(context2, 0, 0, 150, 200, b, tagFont[d], wo[d][k])),
        l.desent > f && (f = l.desent);
    l = measureTextH_W(context2, 0, 0, 350, 200, b, tagFont[d], wo[d]);
    p = l.height / 2 + 2;
    n = a / 2 + 2;
    m = f;
    m = [
      [-n, -p + m],
      [n, -p + m],
      [n, p + m],
      [-n, p + m],
    ];
    fixDef.shape.SetAsArray(createVerticesArray(m), 4);
    bodyheight[d] = l.height / 2;
    c.CreateFixture(fixDef);
  }
}
function createVerticesArray(a) {
  for (var b = [], c = 0; c < a.length; c++) {
    var d = new b2Vec2(a[c][0] / drawScale, a[c][1] / drawScale);
    b.push(d);
  }
  return b;
}
function preserveConstrain() {
  for (var a = 0; a < constrainBody.length; a++) {
    for (var b = [], c = 0; c < a; c++)
      for (var d = 0; d < constrainBody[c].length; d++)
        b.push(constrainBody[c][d]);
    if (0 <= constrainBody[a][0]) {
      constrainBody[a].sort(sortNumber);
      for (
        var c = constrainBody[a][0], d = findNeighbor(!0, c), e = 1;
        e < constrainBody[a].length;
        e++
      ) {
        for (
          var f = constrainBody[a][e], g = 999, h = -1, k = !1, l = 0.1;
          -1 == h;

        ) {
          for (var n = 0; n < d.length; n++)
            if (
              (f == d[n] && (k = !0),
              -1 == b.indexOf(d[n]) && Math.abs(we[f] - we[d[n]]) < l)
            ) {
              var p = !0;
              bodyList[f].IsFixedRotation() &&
                0 != bodyList[d[n]].GetAngle() &&
                (p = !1);
              p &&
                ((p = Math.abs(bodywidth[f] - bodywidth[d[n]])),
                p < g && ((g = p), (h = d[n])));
            }
          l += 0.1;
        }
        k || changeBodyPosition(f, h);
        tagColor[f] = tagColor[c];
      }
    }
  }
}
function changeBodyPosition(a, b) {
  var c = bodyList[a],
    d = bodyList[b];
  c.IsFixedRotation() &&
    0 != c.GetAngle() &&
    (d.SetAngle(c.GetAngle()), c.SetAngle(0));
  for (var e = c.GetFixtureList(); e; e = e.m_next) e.SetSensor(!0);
  for (e = d.GetFixtureList(); e; e = e.m_next) e.SetSensor(!0);
  var e = c.GetPosition().x,
    f = c.GetPosition().y,
    g = d.GetPosition().x,
    h = d.GetPosition().y;
  c.SetPosition(new b2Vec2(g, h));
  d.SetPosition(new b2Vec2(e, f));
  for (e = c.GetFixtureList(); e; e = e.m_next) e.SetSensor(!1);
  for (e = d.GetFixtureList(); e; e = e.m_next) e.SetSensor(!1);
}
function movetonearNeighbor(a, b, c) {
  a = bodyList[a];
  b = bodyList[b];
  var d = bodyList[c];
  a.IsFixedRotation() && 0 != a.GetAngle() && a.SetAngle(0);
  c = b.GetPosition().x;
  b = b.GetPosition().y;
  d.GetPosition();
  d = d.GetPosition().y;
  a.SetPosition(new b2Vec2(c, b - 0.5 * (b - d)));
}
function createAngle() {
  var a = parseInt(6 * Math.random());
  return fw[a];
}
function createColor() {
  var a = parseInt(6 * Math.random());
  return color[colorthemenum][a];
}
function createOffSet() {
  return 0.5 <= Math.random() ? 70 * Math.random() : 70 * -Math.random();
}
function placeBody(a, b) {
  var c = 0,
    d = [],
    e = null,
    f = 1600;
  b && (f = 1e3);
  for (var g = 0; g < f; g++) {
    var h = getSpiralNudger(g),
      k = a.GetPosition().x + h.x / drawScale / 2,
      h = a.GetPosition().y + h.y / drawScale / 2;
    a.SetPosition(new b2Vec2(k, h));
    var l = a.GetPosition();
    if (
      !(
        0 > l.x ||
        0 > l.y ||
        l.x > canvaswidth ||
        l.y > canvasheight ||
        (null != e && isOverlap(e, a, b))
      )
    ) {
      for (var l = !1, n = 0; n < drawnBody.length; n++)
        if (isOverlap(drawnBody[n], a, b)) {
          l = !0;
          e = drawnBody[n];
          break;
        }
      if (!l)
        if (b)
          if (20 > c) (d[c] = { x: k, y: h }), c++;
          else return d;
        else return drawnBody.push(a), { x: k, y: h };
    }
  }
  return b ? d : !1;
}
function isOverlap(a, b, c) {
  var d = a.GetPosition(),
    e = b.GetPosition();
  a = a.GetID();
  b = b.GetID();
  0.5 <= we[a] && (d = CorrectCenter(a, d));
  0.5 <= we[b] && (e = CorrectCenter(b, e));
  d = getPoint(d, a, c);
  c = getPoint(e, b, c);
  return doPolygonsIntersect(d, c);
}
function doPolygonsIntersect(a, b) {
  var c = [a, b],
    d,
    e,
    f,
    g;
  for (d = 0; d < c.length; d++) {
    var h = c[d];
    for (e = 0; e < h.length; e++) {
      var k = h[e];
      var l = h[(e + 1) % h.length];
      var n = l.y - k.y,
        p = k.x - l.x;
      k = l = void 0;
      for (f = 0; f < a.length; f++) {
        var m = n * a[f].x + p * a[f].y;
        if (void 0 == k || m < k) k = m;
        if (void 0 == l || m > l) l = m;
      }
      var q = (g = void 0);
      for (f = 0; f < b.length; f++) {
        m = n * b[f].x + p * b[f].y;
        if (void 0 == q || m < q) q = m;
        if (void 0 == g || m > g) g = m;
      }
      if (l < q || g < k) return !1;
    }
  }
  return !0;
}
function getPoint(a, b, c) {
  var d = bodyList[b].GetPosition(),
    e = 1;
  c && (e = 0.9);
  a = [
    {
      x: a.x * drawScale - bodywidth[b] * e,
      y: a.y * drawScale - bodyheight[b] * e,
    },
    {
      x: a.x * drawScale + bodywidth[b] * e,
      y: a.y * drawScale - bodyheight[b] * e,
    },
    {
      x: a.x * drawScale + bodywidth[b] * e,
      y: a.y * drawScale + bodyheight[b] * e,
    },
    {
      x: a.x * drawScale - bodywidth[b] * e,
      y: a.y * drawScale + bodyheight[b] * e,
    },
  ];
  b = bodyList[b].GetAngle();
  if (0 != b)
    for (c = 0; c < a.length; c++)
      (e =
        (a[c].x - d.x * drawScale) * Math.cos(b) -
        (a[c].y - d.y * drawScale) * Math.sin(b) +
        d.x * drawScale),
        (a[c].y =
          (a[c].x - d.x * drawScale) * Math.sin(b) +
          (a[c].y - d.y * drawScale) * Math.cos(b) +
          d.y * drawScale),
        (a[c].x = e);
  return a;
}
function getSpiralNudger(a) {
  var b = powerMap(0.5, a, 0, 600, 1, 100);
  a *= powerMap(1, a, 0, 600, 0.5, 0.3);
  return new b2Vec2(Math.cos(a) * b, Math.sin(a) * b);
}
function powerMap(a, b, c, d, e, f) {
  return (f - e) * Math.pow(b / (d - c), a) + e;
}
function debugDraw() {
  var a = new b2DebugDraw();
  context = document.getElementById("canvas").getContext("2d");
  a.SetSprite(context);
  a.SetDrawScale(30);
  a.SetFillAlpha(0.5);
  a.SetLineThickness(1);
  a.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit);
  world.SetDebugDraw(a);
}
var mouseX,
  mouseY,
  premouseX = void 0,
  premouseY = void 0,
  mousePVec,
  isMouseMove,
  isMouseRotate,
  selectedBody,
  multicenter,
  hasselect = { flag: !1, time: 0 },
  preselectBody,
  mrRadian,
  firstrotate = !0,
  scalestart,
  scaleend,
  preBodypos,
  mousevector,
  rotationstart,
  rotationend = void 0,
  mousestart,
  mouseend = void 0,
  prefingerpositon = [],
  premulpos = [],
  judge,
  canvasPosition,
  canvas = document.getElementById("canvas");
canvas.addEventListener("mousedown", handleMouseDown, !0);
canvas.addEventListener("touchstart", handleMouseDown, !0);
canvas.addEventListener("mouseup", handleMouseUp, !0);
canvas.addEventListener("touchend", handleMouseUp, !0);
canvas.addEventListener("mousewheel", handleMousescroll, !0);
var mc = new Hammer.Manager(canvas),
  pinch = new Hammer.Pinch(),
  rotate = new Hammer.Rotate();
pinch.recognizeWith(rotate);
mc.add([pinch, rotate]);
mc.on("rotatestart rotatemove", handleTouchRotate);
mc.on("pinchstart pinchmove", handleTouchPinch);
document.onkeydown = function (a) {
  (a = a || window.event) && 16 == a.keyCode && (isshift = !0);
  a && 17 == a.keyCode && (isctrl = !0);
  a && 90 == a.keyCode && (isz = !0);
};
document.onkeyup = function (a) {
  (a = a || window.event) && 16 == a.keyCode && (isshift = !1);
  a && 17 == a.keyCode && (isctrl = !1);
};
function handleMouseDown(a) {
  if (a.clientX) {
    var b = a.clientX;
    var c = a.clientY;
  } else if (
    a.changedTouches &&
    0 < a.targetTouches.length &&
    1 == a.targetTouches.length
  )
    (c = a.changedTouches[a.changedTouches.length - 1]),
      (b = c.clientX),
      (c = c.clientY);
  else return;
  canvasPosition = getElementPosition();
  mouseX = (b - canvasPosition.x) / 30;
  mouseY = (c - canvasPosition.y) / 30;
  premouseX = mouseX;
  premouseY = mouseY;
  a.preventDefault();
  if (!isctrl && 0 == multibodies.length)
    if (hasselect.flag && 1 <= hasselect.time)
      if ((b = getBodyAtMouse()))
        if (ifaddconstrain) {
          c = constrainBody.length - 1;
          var d = b.GetID();
          b = !1;
          for (e = 0; e < constrainBody[c].length; e++)
            constrainBody[c][e] == d && (constrainBody[c].remove(d), (b = !0));
          b || constrainBody[c].push(d);
        } else {
          if (1024 == d)
            for (var e = 0; e < bodyList.length; e++)
              bodyList[e].SetFixedRotation(!1);
          preBodypos = { x: b.GetWorldCenter().x, y: b.GetWorldCenter().y };
          mousevector = {
            x: b.GetWorldCenter().x - mouseX,
            y: b.GetWorldCenter().y - mouseY,
          };
          SavePreEditPosition();
          firstrotate = !0;
          b.SetType(b2Body.b2_staticBody);
          preselectBody == b
            ? (hasselect.time++,
              ifaddconstrain ||
                (handleMouseMove(a),
                canvas.addEventListener("mousemove", handleMouseMove, !0),
                canvas.addEventListener("touchmove", handleMouseMove, !0)))
            : ((preselectBody = b),
              $("#wordfont").selectpicker(
                "val",
                Allfont.indexOf(tagFont[b.GetID()])
              ),
              (document.getElementById("wordColor").style.backgroundColor =
                tagColor[b.GetID()]),
              (document.getElementById("wordColor").value = "word color"),
              (ifedit = hasselect.flag = !0),
              (hasselect.time = 1),
              handleMouseMove(a),
              canvas.addEventListener("mousemove", handleMouseMove, !0),
              canvas.addEventListener("touchmove", handleMouseMove, !0));
        }
      else
        ifaddconstrain ||
          (handleMouseUp(), (hasselect.flag = !1), (hasselect.time = 0)),
          LogOff(),
          (document.getElementById("wordedittools").style.display = "none");
    else
      (b = getBodyAtMouse())
        ? SingleWordPick(b, a)
        : ((ismousemulti = !0),
          (mousestart = { x: mouseX, y: mouseY }),
          canvas.addEventListener("mousemove", handleMouseMove, !0),
          canvas.addEventListener("touchmove", handleMouseMove, !0));
  isctrl &&
    (b = getBodyAtMouse()) &&
    (hasselect.flag &&
      ((d = preselectBody.GetID()),
      multibodies.push(d),
      handleMouseUp(),
      (hasselect.flag = !1),
      (hasselect.time = 0),
      LogOff(),
      (document.getElementById("wordedittools").style.display = "none")),
    (d = b.GetID()),
    -1 != multibodies.indexOf(d) ? multibodies.remove(d) : multibodies.push(d),
    FindMultiCenter());
  if (!isctrl && 0 != multibodies.length)
    if ((b = getBodyAtMouse()))
      if (((d = b.GetID()), -1 == multibodies.indexOf(d)))
        (multibodies = []), (multicenter = void 0), SingleWordPick(b, a);
      else {
        firstrotate = !0;
        SavePreEditPosition();
        preBodypos = { x: b.GetWorldCenter().x, y: b.GetWorldCenter().y };
        mousevector = {
          x: b.GetWorldCenter().x - mouseX,
          y: b.GetWorldCenter().y - mouseY,
        };
        for (e = 0; e < multibodies.length; e++)
          bodyList[multibodies[e]].SetType(b2Body.b2_staticBody);
        handleMouseMove(a);
        canvas.addEventListener("mousemove", handleMouseMove, !0);
      }
    else (multibodies = []), (multicenter = void 0), LogOff();
  "block" == $("#ShowWordsList").css("display") && CreateWordTable();
  if (
    (hasselect.flag && 1024 != selectedBody.GetID()) ||
    (0 != multibodies.length && multibodies.length != wordnum)
  )
    judge = setInterval(function () {
      var a;
      if (
        0.01 > Math.abs(selectedBody.GetWorldCenter().x - preBodypos.x) &&
        0.01 > Math.abs(selectedBody.GetWorldCenter().y - preBodypos.y)
      )
        if (
          (void 0 == preMovePosition[0] && SaveMovePositon(),
          0 == multibodies.length)
        ) {
          for (a = selectedBody.GetFixtureList(); a; a = a.m_next)
            a.SetSensor(!1);
          ifattract[selectedBody.GetID()] = !1;
        } else
          for (var c = 0; c < multibodies.length; c++) {
            var b = multibodies[c];
            for (a = bodyList[b].GetFixtureList(); a; a = a.m_next)
              a.SetSensor(!1);
            ifattract[b] = !1;
          }
    }, 150);
}
function FindMultiCenter() {
  for (
    var a = 0, b = 0, c = 1500, d = 1500, e = 0;
    e < multibodies.length;
    e++
  ) {
    var f = multibodies[e],
      g = bodyList[f].GetPosition();
    0.5 < we[f] && (g = CorrectCenter(f, g));
    var h = bodyList[f].GetPosition(),
      g = [
        {
          x: g.x * drawScale - bodywidth[f],
          y: g.y * drawScale - bodyheight[f],
        },
        {
          x: g.x * drawScale + bodywidth[f],
          y: g.y * drawScale - bodyheight[f],
        },
        {
          x: g.x * drawScale + bodywidth[f],
          y: g.y * drawScale + bodyheight[f],
        },
        {
          x: g.x * drawScale - bodywidth[f],
          y: g.y * drawScale + bodyheight[f],
        },
      ],
      f = bodyList[f].GetAngle();
    if (0 != f)
      for (var k = 0; k < g.length; k++) {
        var l =
          (g[k].x - h.x * drawScale) * Math.cos(f) -
          (g[k].y - h.y * drawScale) * Math.sin(f) +
          h.x * drawScale;
        g[k].y =
          (g[k].x - h.x * drawScale) * Math.sin(f) +
          (g[k].y - h.y * drawScale) * Math.cos(f) +
          h.y * drawScale;
        g[k].x = l;
      }
    for (k = 0; k < g.length; k++)
      g[k].x > b && (b = g[k].x),
        g[k].x < d && (d = g[k].x),
        g[k].y > a && (a = g[k].y),
        g[k].y < c && (c = g[k].y);
  }
  multicenter = {
    x: ((b - d) / 2 + d) / drawScale,
    y: ((a - c) / 2 + c) / drawScale,
  };
}
function LogOff() {
  preBodypos = premouseY = premouseX = mouseY = mouseX = void 0;
  firstrotate = !0;
  mrRadian = selectedBody = preselectBody = void 0;
  premulpos = [];
  multicenter = void 0;
}
function SingleWordPick(a, b) {
  hasselect.flag = !0;
  preselectBody = a;
  var c = a.GetID();
  hasselect.time++;
  if (ifaddconstrain) {
    var d = constrainBody.length,
      e = [];
    e.push(c);
    constrainBody[d] = e;
  } else {
    if (1024 == c)
      for (c = 0; c < bodyList.length; c++) bodyList[c].SetFixedRotation(!1);
    preBodypos = { x: a.GetWorldCenter().x, y: a.GetWorldCenter().y };
    mousevector = {
      x: a.GetWorldCenter().x - mouseX,
      y: a.GetWorldCenter().y - mouseY,
    };
    SavePreEditPosition();
    a.SetType(b2Body.b2_staticBody);
    ifedit = !0;
    document.getElementById("wordedittools").style.display = "";
    $("#wordfont").selectpicker("val", Allfont.indexOf(tagFont[a.GetID()]));
    document.getElementById("wordColor").style.backgroundColor =
      tagColor[a.GetID()];
    document.getElementById("wordColor").value = "word color";
    handleMouseMove(b);
    canvas.addEventListener("mousemove", handleMouseMove, !0);
    canvas.addEventListener("touchmove", handleMouseMove, !0);
  }
}
function handleMouseUp() {
  if (hasselect.flag) {
    canvas.removeEventListener("mousemove", handleMouseMove, !0);
    canvas.removeEventListener("touchmove", handleMouseMove, !0);
    isMouseRotate = isMouseMove = !1;
    AddForce = !0;
    preselectBody.SetType(b2Body.b2_dynamicBody);
    var a = preselectBody.GetFixtureList();
    if (1 == a.IsSensor())
      for (a = preselectBody.GetFixtureList(); a; a = a.m_next) a.SetSensor(!1);
    var b = preselectBody.GetID();
    var c = preselectBody.GetContactList();
    ifattract[b] = !1;
    if (!isz) {
      for (; c; ) {
        if (c.contact.IsTouching()) {
          ifattract[b] = !0;
          break;
        }
        c = c.next;
      }
      ifattract[b] && (iternum = 0);
    }
  }
  if (0 != multibodies.length) {
    canvas.removeEventListener("mousemove", handleMouseMove, !0);
    isMouseRotate = isMouseMove = !1;
    AddForce = !0;
    for (var d = !1, b = 0; b < multibodies.length; b++) {
      c = multibodies[b];
      bodyList[c].SetType(b2Body.b2_dynamicBody);
      a = bodyList[c].GetFixtureList();
      if (1 == a.IsSensor())
        for (a = bodyList[c].GetFixtureList(); a; a = a.m_next) a.SetSensor(!1);
      ifattract[c] = !1;
      for (c = bodyList[c].GetContactList(); c; ) {
        if (c.contact.IsTouching()) {
          d = !0;
          break;
        }
        c = c.next;
      }
    }
    for (b = 0; b < multibodies.length; b++)
      (c = multibodies[b]), (ifattract[c] = d);
    d && (iternum = 0);
  }
  ismousemulti = isz = !1;
  mousevector = void 0;
  if (mouseend) {
    d = mousestart.x > mouseend.x ? mousestart.x : mouseend.x;
    c = mousestart.x <= mouseend.x ? mousestart.x : mouseend.x;
    a = mousestart.y > mouseend.y ? mousestart.y : mouseend.y;
    for (
      var e = mousestart.y <= mouseend.y ? mousestart.y : mouseend.y, b = 0;
      b < wordnum;
      b++
    ) {
      var f = bodyList[b].GetPosition();
      f.x > c && f.x < d && f.y > e && f.y < a && multibodies.push(b);
    }
    FindMultiCenter();
  }
  mouseend = mousestart = void 0;
  preMovePosition = [];
  clearInterval(judge);
  $("#wordsFont").selectpicker(
    "val",
    Allfont.indexOf(tagFont[multibodies[multibodies.length - 1]])
  );
  document.getElementById("wordsColor").style.backgroundColor =
    tagColor[multibodies[multibodies.length - 1]];
  document.getElementById("wordsColor").value = "words color";
  "block" == $("#ShowWordsList").css("display") && CreateWordTable();
}
function handleMouseMove(a) {
  if (a.clientX) {
    var b = a.clientX;
    var c = a.clientY;
  } else if (a.changedTouches && 0 < a.changedTouches.length)
    (c = a.changedTouches[a.changedTouches.length - 1]),
      (b = c.clientX),
      (c = c.clientY);
  else return;
  mouseX = (b - canvasPosition.x) / 30;
  mouseY = (c - canvasPosition.y) / 30;
  a.preventDefault();
  if (selectedBody) {
    if (0 == multibodies.length && 1024 != preselectBody.GetID()) {
      if (
        (preselectBody.SetAwake(!0),
        preselectBody.SetAngularVelocity(0),
        preselectBody.SetLinearVelocity(new b2Vec2(0, 0)),
        !(
          mouseX >= premouseX - 2 / 30 &&
          mouseX <= premouseX + 2 / 30 &&
          mouseY >= premouseY - 2 / 30 &&
          mouseY <= premouseY + 2 / 30
        ))
      )
        for (
          preMovePosition[0]
            ? LoadMovePosition(preselectBody.GetID(), !1)
            : ifattract[preselectBody.GetID()] && (iternum = 0),
            c = preselectBody.GetFixtureList();
          c;
          c = c.m_next
        )
          c.SetSensor(!0);
    } else {
      for (b = 0; b < multibodies.length; b++)
        (c = multibodies[b]),
          bodyList[c].SetAwake(!0),
          bodyList[c].SetAngularVelocity(0),
          bodyList[c].SetLinearVelocity(new b2Vec2(0, 0));
      if (
        !(
          mouseX >= premouseX - 2 / 30 &&
          mouseX <= premouseX + 2 / 30 &&
          mouseY >= premouseY - 2 / 30 &&
          mouseY <= premouseY + 2 / 30
        )
      )
        for (
          preMovePosition[0] ? LoadMovePosition(void 0, !0) : (iternum = 0),
            b = 0;
          b < multibodies.length;
          b++
        )
          for (
            c = multibodies[b], c = bodyList[c].GetFixtureList();
            c;
            c = c.m_next
          )
            c.SetSensor(!0);
    }
    premouseX = mouseX;
    premouseY = mouseY;
    if (a.targetTouches) {
      if (
        (1 == a.targetTouches.length && hasselect.flag && (isMouseMove = !0),
        2 == a.targetTouches.length && hasselect.flag)
      )
        for (c = preselectBody.GetFixtureList(); c; c = c.m_next)
          c.SetSensor(!1);
    } else {
      if (
        (0 == a.button && hasselect.flag) ||
        (0 == a.button && 0 != multibodies.length)
      )
        isMouseMove = !0;
      if (
        (2 == a.button && hasselect.flag) ||
        (2 == a.button && 0 != multibodies.length)
      ) {
        if (0 == multibodies.length)
          for (c = preselectBody.GetFixtureList(); c; c = c.m_next)
            c.SetSensor(!1);
        else
          for (b = 0; b < multibodies.length; b++)
            for (
              c = multibodies[b], c = bodyList[c].GetFixtureList();
              c;
              c = c.m_next
            )
              c.SetSensor(!1);
        isMouseRotate = !0;
      }
    }
  }
  ismousemulti && (mouseend = { x: mouseX, y: mouseY });
}
function handleMousescroll(a) {
  var b;
  a = a || window.event;
  if (hasselect.flag) {
    var c = selectedBody.GetID();
    if (1024 == c) {
      if (0 > a.wheelDelta) {
        var d = 0.9;
        for (b = pushingbar.GetFixtureList(); b; b = b.m_next) {
          var e = b.GetShape();
          var f = e.GetVertices();
          for (var g in f) (e = f[g]), (e.x *= d), (e.y *= d);
        }
      } else
        for (d = 1.1, b = pushingbar.GetFixtureList(); b; b = b.m_next)
          for (g in ((e = b.GetShape()), (f = e.GetVertices()), f))
            (e = f[g]), (e.x *= d), (e.y *= d);
      bodyheight[c] *= d;
      bodywidth[c] *= d;
    } else
      a.wheelDelta &&
        (0 > a.wheelDelta
          ? ((b = 0.95 * we[c]), ScaleWords(c, b))
          : ((b = 1.05 * we[c]),
            0.5 > b || (0.5 <= b && 1 == ifsubdivided[c])
              ? ScaleWords(c, b)
              : ((d = {
                  x: bodyList[c].GetPosition().x,
                  y: bodyList[c].GetPosition().y,
                }),
                (g = bodyList[c].GetAngle()),
                (e = bodyList[c].IsFixedRotation()),
                world.DestroyBody(bodyList[c]),
                (we[c] = b),
                ReCreateBody(d, g, e, c, !1, !0),
                (ifsubdivided[c] = !0)))),
        ifattract[c] && (iternum = 0);
  }
  a.preventDefault && a.preventDefault();
}
function ScaleWords(a, b) {
  context2.font =
    (maxfontsize - minfontsize) * b + minfontsize + "px " + tagFont[a];
  var c = context2.measureText(wo[a]).width / 2,
    d = c / bodywidth[a],
    e;
  for (e = bodyList[a].GetFixtureList(); e; e = e.m_next) {
    var f = e.GetShape().GetVertices(),
      g;
    for (g in f) {
      var h = f[g];
      h.x *= d;
      h.y *= d;
    }
  }
  bodyheight[a] *= d;
  bodywidth[a] = c;
  we[a] = b;
}
function handleTouchRotate(a) {
  if (hasselect.flag) {
    isMouseRotate = isMouseMove = !1;
    var b;
    for (b = selectedBody.GetFixtureList(); b; b = b.m_next) b.SetSensor(!1);
    switch (a.type) {
      case "rotatestart":
        rotationstart = a.rotation;
        break;
      case "rotatemove":
        (rotationend = a.rotation - rotationstart),
          (rotationstart = a.rotation),
          selectedBody.SetAngularVelocity(0),
          selectedBody.SetLinearVelocity(new b2Vec2(0, 0)),
          selectedBody.SetAwake(!0),
          selectedBody.m_force.SetZero(),
          (b = selectedBody.GetAngle()),
          selectedBody.SetAngle(b + (rotationend * Math.PI) / 180),
          (prefingerpositon[0] = {
            x: a.pointers[0].pageX,
            y: a.pointers[0].pageY,
          }),
          (prefingerpositon[1] = {
            x: a.pointers[1].pageX,
            y: a.pointers[1].pageY,
          });
    }
  }
}
function handleTouchPinch(a) {
  var b;
  if (hasselect.flag) {
    var c = a.pointers,
      d = 1;
    isMouseRotate = isMouseMove = !1;
    for (b = selectedBody.GetFixtureList(); b; b = b.m_next) b.SetSensor(!1);
    switch (a.type) {
      case "pinchstart":
        scalestart = Math.sqrt(
          (c[0].pageX - c[1].pageX) * (c[0].pageX - c[1].pageX) +
            (c[0].pageY - c[1].pageY) * (c[0].pageY - c[1].pageY)
        );
        break;
      case "pinchmove":
        (scaleend = Math.sqrt(
          (c[0].pageX - c[1].pageX) * (c[0].pageX - c[1].pageX) +
            (c[0].pageY - c[1].pageY) * (c[0].pageY - c[1].pageY)
        )),
          (d = scaleend / scalestart),
          (scalestart = scaleend);
    }
    a = selectedBody.GetID();
    if (1024 == a) {
      for (b = pushingbar.GetFixtureList(); b; b = b.m_next) {
        var c = b.GetShape().GetVertices();
        for (f in c) {
          var e = c[f];
          e.x *= d;
          e.y *= d;
        }
      }
      bodyheight[a] *= d;
      bodywidth[a] *= d;
    } else {
      b = we[a] * d;
      if (0.5 > b || (0.5 <= b && 1 == ifsubdivided[a])) ScaleWords(a, b);
      else {
        d = { x: bodyList[a].GetPosition().x, y: bodyList[a].GetPosition().y };
        var f = bodyList[a].GetAngle();
        c = bodyList[a].IsFixedRotation();
        world.DestroyBody(bodyList[a]);
        we[a] = b;
        ReCreateBody(d, f, c, a, !1, !0);
        ifsubdivided[a] = !0;
      }
      ifattract[a] && (iternum = 0);
    }
  }
}
function getBodyAtMouse() {
  mousePVec = new b2Vec2(mouseX, mouseY);
  var a = new b2AABB();
  a.lowerBound.Set(mouseX - 0.001, mouseY - 0.001);
  a.upperBound.Set(mouseX + 0.001, mouseY + 0.001);
  selectedBody = null;
  world.QueryAABB(getBodyCB, a);
  return selectedBody;
}
function getBodyCB(a) {
  return 0 <= a.GetBody().GetID() &&
    a.GetShape().TestPoint(a.GetBody().GetTransform(), mousePVec)
    ? ((selectedBody = a.GetBody()), !1)
    : !0;
}
var temPosition = void 0;
function findNeighbor(a, b) {
  var c = 0;
  a && (c = b);
  for (c; c < wordnum; c++) {
    new b2Vec2();
    var d = a ? bodyList[c].GetWorldCenter() : prePosition[c];
    0.5 <= we[c] && (d = CorrectCenter(c, d));
    for (var e = [], f = 0; f < wordnum; f++)
      if (f != c) {
        var g = a ? bodyList[f].GetWorldCenter() : prePosition[f];
        0.5 <= we[f] && (g = CorrectCenter(f, g));
        var h = !1;
        if (havedecent) {
          g = world.RayCastAll(d, g);
          for (var k, l = 0; l < g.length; l++)
            if (((k = g[l].GetBody().GetID()), k != c && k != f)) {
              h = !0;
              break;
            }
        } else
          for (l = 0; l < wordnum; l++)
            if (l != f && l != c) {
              var n = bodyList[l].GetPosition();
              k = bodyList[l].GetWorldCenter();
              0.5 <= we[l] && (k = CorrectCenter(l, k));
              k = [
                {
                  x: k.x * drawScale - bodywidth[l],
                  y: k.y * drawScale - bodyheight[l],
                },
                {
                  x: k.x * drawScale + bodywidth[l],
                  y: k.y * drawScale - bodyheight[l],
                },
                {
                  x: k.x * drawScale + bodywidth[l],
                  y: k.y * drawScale + bodyheight[l],
                },
                {
                  x: k.x * drawScale - bodywidth[l],
                  y: k.y * drawScale + bodyheight[l],
                },
              ];
              var p = bodyList[l].GetAngle();
              if (0 != p)
                for (var m = 0; m < k.length; m++) {
                  var q =
                    (k[m].x - n.x * drawScale) * Math.cos(p) -
                    (k[m].y - n.y * drawScale) * Math.sin(p) +
                    n.x * drawScale;
                  k[m].y =
                    (k[m].x - n.x * drawScale) * Math.sin(p) +
                    (k[m].y - n.y * drawScale) * Math.cos(p) +
                    n.y * drawScale;
                  k[m].x = q;
                }
              n = { x: d.x * drawScale, y: d.y * drawScale };
              p = { x: g.x * drawScale, y: g.y * drawScale };
              for (m = 0; 4 > m; m++)
                if (0 != segmentsIntr(n, p, k[m], 3 == m ? k[0] : k[m + 1])) {
                  h = !0;
                  break;
                }
            }
        h || e.push(f);
      }
    if (a) return e;
    wordsNeighbor[c] = e;
  }
  temPosition = void 0;
}
function calR(a) {
  for (
    var b = [
        [0, 0],
        [0, 0],
      ],
      c = 0;
    c < wordsNeighbor[a].length;
    c++
  )
    var d = wordsNeighbor[a][c],
      e = [
        [prePosition[a].x - prePosition[d].x],
        [prePosition[a].y - prePosition[d].y],
      ],
      f = bodyList[a].GetPosition(),
      d = bodyList[d].GetPosition(),
      e = numeric.dot(e, numeric.transpose([[f.x - d.x], [f.y - d.y]])),
      b = numeric.add(b, e);
  for (c = 0; 2 > c; c++)
    for (e = 0; 2 > e; e++) b[c][e] = parseFloat(b[c][e].toFixed(4));
  c = numeric.svd(b);
  a = numeric.transpose(c.U);
  b = c.V;
  e = c.S;
  a = numeric.dot(b, a);
  0 >= numeric.det(a) &&
    ((a =
      e[0] > e[1]
        ? [
            [c.U[0][0], -c.U[0][1]],
            [c.U[1][0], -c.U[1][1]],
          ]
        : [
            [-c.U[0][0], c.U[0][1]],
            [-c.U[1][0], c.U[1][1]],
          ]),
    (a = numeric.dot(b, numeric.transpose(a))));
  for (c = 0; 2 > c; c++)
    for (e = 0; 2 > e; e++)
      if (!a[c][e] && 0 != a[c][e])
        return [
          [1, 0],
          [0, 1],
        ];
  return a;
}
function calattractF(a) {
  var b = { x: 0, y: 0 };
  if (wordsNeighbor[a])
    for (var c = 0; c < wordsNeighbor[a].length; c++) {
      var d = wordsNeighbor[a][c],
        e = bodyList[a].GetMass(),
        f = bodyList[d].GetMass(),
        g = bodyList[a].GetWorldCenter(),
        h = bodyList[d].GetWorldCenter(),
        d = new b2Vec2(0, 0);
      d.Add(h);
      d.Subtract(g);
      g = d.Length();
      d.Normalize();
      d.Multiply((e * f) / (g * g));
      b.x += d.x;
      b.y += d.y;
    }
  return b;
}
function drawText() {
  Ifdebug ||
    (context.clearRect(0, 0, context.canvas.width, context.canvas.height),
    context.save(),
    (context.fillStyle = backgroundcolor),
    context.fillRect(0, 0, canvaswidth, canvasheight),
    context.restore());
  pushingbar &&
    (context.save(),
    context.translate(
      pushingbar.GetPosition().x * drawScale,
      pushingbar.GetPosition().y * drawScale
    ),
    context.rotate(pushingbar.GetAngle()),
    (context.fillStyle = "#6495ED"),
    context.fillRect(
      -bodywidth[1024],
      -bodyheight[1024],
      2 * bodywidth[1024],
      2 * bodyheight[1024]
    ),
    context.restore());
  if (bodyList[0])
    for (var a = 0; a < wordnum; a++) {
      var b = bodyList[a].GetPosition();
      var c = bodyList[a].GetAngle();
      context.save();
      context.translate(b.x * drawScale, b.y * drawScale);
      context.rotate(c);
      var d = (maxfontsize - minfontsize) * we[a] + minfontsize;
      if (seebox) {
        context.fillStyle = "rgba(255,0,0,0.15)";
        var e = bodywidth[a];
        var f = bodyheight[a];
        0.5 <= we[a]
          ? context.fillRect(-e, -f + d / 2 - f + bodydesent[a], 2 * e, 2 * f)
          : context.fillRect(-e, -f, 2 * e, 2 * f);
        context.lineWidth = 1;
        context.strokeStyle = "rgb(255,0,0)";
        0.5 <= we[a]
          ? context.strokeRect(-e, -f + d / 2 - f + bodydesent[a], 2 * e, 2 * f)
          : context.strokeRect(-e, -f, 2 * e, 2 * f);
      }
      context.font = d + "px " + tagFont[a];
      context.fillStyle = tagColor[a];
      context.textAlign = "center";
      0.5 <= we[a]
        ? context.fillText(wo[a], 0, d / 2)
        : context.fillText(wo[a], 0, bodyheight[a]);
      context.restore();
    }
  for (f = 0; f < animation.length; f++)
    (a = animation[f].id),
      (b = bodyList[a].GetPosition()),
      (c = bodyList[a].GetAngle()),
      context.save(),
      context.translate(b.x * drawScale, b.y * drawScale),
      context.rotate(c),
      (d = (maxfontsize - minfontsize) * we[a] + minfontsize),
      (context.font = d + "px " + tagFont[a]),
      (context.fillStyle = tagColor[a]),
      (context.textAlign = "center"),
      0.5 <= we[a]
        ? context.fillText(wo[a], 0, d / 2)
        : context.fillText(wo[a], 0, bodyheight[a]),
      context.restore();
  for (a = 0; a < multibodies.length; a++) {
    var g = multibodies[a];
    c = bodyList[g].GetAngle();
    b = bodyList[g].GetWorldCenter();
    e = bodywidth[g];
    f = bodyheight[g];
    d = (maxfontsize - minfontsize) * we[g] + minfontsize;
    context.save();
    context.translate(b.x * drawScale, b.y * drawScale);
    context.rotate(c);
    context.strokeStyle = "#ff00ff";
    0.5 <= we[g]
      ? context.strokeRect(-e, -f + d / 2 - f + bodydesent[g], 2 * e, 2 * f)
      : context.strokeRect(-e, -f, 2 * e, 2 * f);
    context.restore();
  }
  hasselect.flag &&
    ((g = selectedBody.GetID()),
    (c = selectedBody.GetAngle()),
    (b = selectedBody.GetPosition()),
    (e = bodywidth[g]),
    (f = bodyheight[g]),
    (d = (maxfontsize - minfontsize) * we[g] + minfontsize),
    context.save(),
    context.translate(b.x * drawScale, b.y * drawScale),
    context.rotate(c),
    (context.strokeStyle = "#ff00ff"),
    0.5 <= we[g]
      ? context.strokeRect(-e, -f + d / 2 - f + bodydesent[g], 2 * e, 2 * f)
      : context.strokeRect(-e, -f, 2 * e, 2 * f),
    context.restore());
  ismousemulti &&
    mousestart &&
    mouseend &&
    (0 < mouseend.x - mousestart.x
      ? ((e = (mouseend.x - mousestart.x) * drawScale),
        (f = (mouseend.y - mousestart.y) * drawScale),
        context.save(),
        (context.fillStyle = "rgba(0,0,0,0.15)"),
        context.fillRect(
          mousestart.x * drawScale,
          mousestart.y * drawScale,
          e,
          f
        ),
        (context.strokeStyle = "#696969"),
        context.strokeRect(
          mousestart.x * drawScale,
          mousestart.y * drawScale,
          e,
          f
        ))
      : ((e = (mousestart.x - mouseend.x) * drawScale),
        (f = (mousestart.y - mouseend.y) * drawScale),
        context.save(),
        (context.fillStyle = "rgba(0,0,0,0.15)"),
        context.fillRect(mouseend.x * drawScale, mouseend.y * drawScale, e, f),
        (context.strokeStyle = "#696969"),
        context.strokeRect(
          mouseend.x * drawScale,
          mouseend.y * drawScale,
          e,
          f
        )),
    context.restore());
  if (hullPoints) {
    context.save();
    context.lineWidth = 2;
    context.beginPath();
    context.strokeStyle = "#ae80ff";
    for (a = 0; a < hullPoints.length; a++)
      0 == a
        ? context.moveTo(hullPoints[a].x, hullPoints[a].y)
        : context.lineTo(hullPoints[a].x, hullPoints[a].y);
    context.closePath();
    context.stroke();
    context.restore();
  }
}
function update() {
  (hasselect.flag || 0 != multibodies.length) &&
    preBodypos &&
    (preBodypos = {
      x: selectedBody.GetWorldCenter().x,
      y: selectedBody.GetWorldCenter().y,
    });
  mouseMoveAndRotate();
  centerAttract();
  forEachAttract();
  AnimationCal();
  world.Step(1 / 60, 10, 10);
  world.DrawDebugData();
  drawText();
  world.ClearForces();
}
var oriRadian;
function mouseMoveAndRotate() {
  if (isMouseMove) {
    selectedBody.SetAngularVelocity(0);
    selectedBody.SetLinearVelocity(new b2Vec2(0, 0));
    selectedBody.SetAwake(!0);
    selectedBody.m_force.SetZero();
    var a = selectedBody.GetPosition().x;
    var b = selectedBody.GetPosition().y;
    var c = mouseX + mousevector.x - a;
    var d = mouseY + mousevector.y - b;
    selectedBody.SetPosition(new b2Vec2(a + c, b + d));
    if (0 != multibodies.length) {
      for (var e = 0; e < multibodies.length; e++) {
        var f = multibodies[e];
        f != selectedBody.GetID() &&
          ((a = bodyList[f].GetPosition().x),
          (b = bodyList[f].GetPosition().y),
          bodyList[f].SetPosition(new b2Vec2(a + c, b + d)));
      }
      multicenter.x += c;
      multicenter.y += d;
    }
  }
  if (isMouseRotate)
    if (firstrotate) {
      oriRadian = selectedBody.GetAngle();
      if (0 < multibodies.length)
        for (
          a = multicenter, oriRadian = [], e = 0;
          e < multibodies.length;
          e++
        )
          (f = multibodies[e]),
            (premulpos[e] = {
              x: bodyList[f].GetPosition().x,
              y: bodyList[f].GetPosition().y,
            }),
            (oriRadian[e] = bodyList[f].GetAngle());
      else a = selectedBody.GetWorldCenter();
      mrRadian = Math.atan2(mouseY - a.y, mouseX - a.x);
      firstrotate = !1;
    } else if (isshift)
      if (0 < multibodies.length) {
        if (
          ((a = multicenter),
          (f = Math.atan2(mouseY - a.y, mouseX - a.x) - mrRadian),
          5 > Math.abs(((180 * f) / Math.PI) % 15))
        ) {
          var g = (15 * Math.floor((180 * f) / Math.PI / 15) * Math.PI) / 180;
          for (e = 0; e < multibodies.length; e++)
            (f = multibodies[e]),
              (b = premulpos[e].x),
              (c = premulpos[e].y),
              (d = (b - a.x) * Math.cos(g) - (c - a.y) * Math.sin(g) + a.x),
              (c = (b - a.x) * Math.sin(g) + (c - a.y) * Math.cos(g) + a.y),
              (b = d),
              bodyList[f].SetPosition(new b2Vec2(b, c)),
              bodyList[f].SetAngle(g + oriRadian[e]);
        }
      } else
        (a = selectedBody.GetWorldCenter()),
          (f = Math.atan2(mouseY - a.y, mouseX - a.x) - mrRadian + oriRadian),
          5 > Math.abs(((180 * f) / Math.PI) % 15) &&
            (selectedBody.SetAngularVelocity(0),
            selectedBody.SetLinearVelocity(new b2Vec2(0, 0)),
            selectedBody.SetAwake(!0),
            selectedBody.m_force.SetZero(),
            (g = 15 * Math.floor((180 * f) / Math.PI / 15)),
            selectedBody.SetAngle((g * Math.PI) / 180));
    else if (0 < multibodies.length) {
      if (((a = multicenter), 0 != multibodies.length))
        for (e = 0; e < multibodies.length; e++)
          (f = multibodies[e]),
            (b = premulpos[e].x),
            (c = premulpos[e].y),
            (g = Math.atan2(mouseY - a.y, mouseX - a.x) - mrRadian),
            (d = (b - a.x) * Math.cos(g) - (c - a.y) * Math.sin(g) + a.x),
            (c = (b - a.x) * Math.sin(g) + (c - a.y) * Math.cos(g) + a.y),
            (b = d),
            bodyList[f].SetPosition(new b2Vec2(b, c)),
            bodyList[f].SetAngle(g + oriRadian[e]);
    } else
      selectedBody.SetAngularVelocity(0),
        selectedBody.SetLinearVelocity(new b2Vec2(0, 0)),
        selectedBody.SetAwake(!0),
        selectedBody.m_force.SetZero(),
        (a = selectedBody.GetWorldCenter()),
        selectedBody.SetAngle(
          Math.atan2(mouseY - a.y, mouseX - a.x) - mrRadian + oriRadian
        );
}
function centerAttract() {
  if (usecenter && AddForce && UserAddForce)
    for (var a = 0; a < wordnum; a++)
      if (ifattract[a])
        for (var b = 0; b < planet.length; b++) {
          var c = bodyList[a].GetWorldCenter(),
            d = planet[b].GetFixtureList().GetShape().GetRadius(),
            e = planet[b].GetWorldCenter(),
            f = new b2Vec2(0, 0);
          f.Add(c);
          f.Subtract(e);
          c = f.Length();
          f.Normalize();
          c <= 55e5 * d &&
            (1 > c
              ? (bodyList[a].SetAngularVelocity(0),
                bodyList[a].SetLinearVelocity(new b2Vec2(0, 0)))
              : (f.NegativeSelf(),
                (d = bodyList[a].GetMass()),
                f.Multiply((centerweight / (iternum + 1)) * d * c * c * 1),
                bodyList[a].ApplyForce(f, bodyList[a].GetWorldCenter())));
        }
  iternum++;
  isFirstContrust &&
    10 == iternum &&
    (preserveConstrain(), (isFirstContrust = !1));
}
function forEachAttract() {
  if (ifstart && AddForce && UserAddForce)
    if (130 > iternum) {
      (2 >= iternum || 0 == iternum % 3) && findNeighbor(!1, void 0);
      for (var a = 0; a < wordnum; a++) {
        var b = calattractF(a),
          b = new b2Vec2(b.x * arapweight, b.y * arapweight);
        if (ifattract[a]) {
          b.Multiply(1 / (iternum + 1));
          bodyList[a].ApplyForce(b, bodyList[a].GetWorldCenter());
          var b = bodyList[a].GetLinearVelocity().x,
            c = bodyList[a].GetLinearVelocity().y;
          45 < iternum &&
            bodyList[a].SetLinearVelocity(new b2Vec2(0.96 * b, 0.96 * c));
          80 < iternum &&
            bodyList[a].SetLinearVelocity(new b2Vec2(0.8 * b, 0.8 * c));
        } else
          bodyList[a].SetAngularVelocity(0),
            bodyList[a].SetLinearVelocity(new b2Vec2(0, 0));
      }
    } else if (bodyList[0])
      for (a = 0; a < wordnum; a++)
        bodyList[a].SetAngularVelocity(0),
          bodyList[a].SetLinearVelocity(new b2Vec2(0, 0));
  UserReWordle && (findNeighbor(!1, void 0), ReWordle(), (UserReWordle = !1));
  if (bodyList[0]) {
    for (a = 0; a < wordnum; a++) prePosition[a] = bodyList[a].GetPosition();
    ifstart = !0;
  }
}
function AnimationCal() {
  if (0 != animation.length)
    for (var a = 0; a < animation.length; a++) {
      var b = animation[a],
        c = 15;
      if (0 == b.editflag || 4 == b.editflag) c = 20;
      if (b.count < c)
        bodyList[b.id].SetPosition(
          new b2Vec2(
            b.startpos.x + ((b.endpos.x - b.startpos.x) * (b.count + 1)) / c,
            b.startpos.y + ((b.endpos.y - b.startpos.y) * (b.count + 1)) / c
          )
        ),
          b.count++;
      else {
        bodyList[b.id].SetAngularVelocity(0);
        bodyList[b.id].SetLinearVelocity(new b2Vec2(0, 0));
        animation.splice(a, 1);
        (0 != b.editflag && 3 != b.editflag) ||
          1 != animation.length ||
          ((iternum = 0), (AddForce = !0));
        if (0 == b.editflag || 2 <= b.editflag)
          for (b = bodyList[b.id].GetFixtureList(); b; b = b.m_next)
            b.SetSensor(!1);
        a--;
      }
    }
}
function segmentsIntr(a, b, c, d) {
  var e = (a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x),
    f = (a.x - d.x) * (b.y - d.y) - (a.y - d.y) * (b.x - d.x);
  if (0 <= e * f) return !1;
  c = (c.x - a.x) * (d.y - a.y) - (c.y - a.y) * (d.x - a.x);
  if (0 <= c * (c + e - f)) return !1;
  e = c / (f - e);
  return { x: a.x + e * (b.x - a.x), y: a.y + e * (b.y - a.y) };
}
function sortNumber(a, b) {
  return a - b;
}
function SaveEditData() {
  for (
    var a = [], b = [], c = [], d = [], e = [], f = [], g = [], h = 0;
    h < wordnum;
    h++
  ) {
    var k = {};
    k.x = bodyList[h].GetPosition().x;
    k.y = bodyList[h].GetPosition().y;
    a.push(k);
    b.push(bodyList[h].GetAngle());
    e[h] = ifattract[h];
    c[h] = tagColor[h];
    d[h] = tagFont[h];
    f[h] = wo[h];
    g[h] = we[h];
  }
  a = {
    word: f,
    weight: g,
    position: a,
    angle: b,
    ifattract: e,
    color: c,
    font: d,
    iffix: bodyList[0].IsFixedRotation(),
    constrain: constrainBody,
    iter: iternum,
  };
  document.getElementById("myTextArea").innerHTML = JSON.stringify(a);
  a = document.createElement("input");
  b =
    document.getElementById("myTextArea").innerHTML ||
    document.getElementById("myTextArea").value;
  a.setAttribute("value", b);
  document.body.appendChild(a);
  a.select();
  document.execCommand("copy");
  document.body.removeChild(a);
  alert(
    "The edit data has been copied to the clipboard. You can paste it to the notepad and save it."
  );
}
function LoadEditData(a) {
  if (void 0 == a) {
    var b = prompt("Please input edit data: ", "");
    a = eval("(" + b + ")");
    isFirstSystemstart &&
      ((world = new b2World(new b2Vec2(0, 0), !0)), createWall(), debugDraw());
    for (b = 0; b < bodyList.length; b++) world.DestroyBody(bodyList[b]);
    hasselect.flag = !1;
    bodyList = [];
    bodywidth = [];
    bodyheight = [];
    prePosition = [];
    drawnBody = [];
    preEditInfo = [];
    preEditPoint = 0;
    preReWordleReason = [-1, -1];
    wo = a.word;
    we = a.weight;
    wordnum = wo.length;
    ifattract = a.ifattract;
    tagColor = a.color;
    tagFont = a.font;
    constrainBody = a.constrain;
    ReSortWoandWe(a.position, a.angle);
    ReCreateObject_Load(a.position, a.angle, a.iffix, a.iter);
  } else {
    isFirstSystemstart &&
      ((world = new b2World(new b2Vec2(0, 0), !0)), createWall(), debugDraw());
    for (b = 0; b < bodyList.length; b++) world.DestroyBody(bodyList[b]);
    $.getJSON("e" + a + ".json", function (a) {
      wo = a.word;
      we = a.weight;
      wordnum = wo.length;
      ifattract = a.ifattract;
      tagColor = a.color;
      tagFont = a.font;
      constrainBody = a.constrain;
      ReSortWoandWe(a.position, a.angle);
      ReCreateObject_Load(a.position, a.angle, a.iffix, a.iter);
      iternum = 0;
    });
  }
  isFirstSystemstart &&
    (window.setInterval(update, 1e3 / 60), (isFirstSystemstart = !1));
}
function ReCreateObject_Load(a, b, c, d) {
  havedecent = !1;
  for (var e = 0; e < wordnum; e++) ReCreateBody(a[e], b[e], c, e, !0, !1);
  iternum = d;
  ifstart = !1;
}
function ReCreateBody(a, b, c, d, e, f) {
  fixDef.isSensor = !1;
  fixDef.density = 1;
  fixDef.friction = 0.2;
  bodyDef.type = b2Body.b2_dynamicBody;
  fixDef.shape = new b2PolygonShape();
  var g = (maxfontsize - minfontsize) * we[d] + minfontsize;
  context2.font = g + "px " + tagFont[d];
  var h = context2.measureText(wo[d]).width;
  bodywidth[d] = h / 2;
  bodyheight[d] = g / 2;
  bodydesent[d] = 0;
  ifsubdivided[d] = !1;
  bodyDef.position.Set(a.x, a.y);
  var k = world.CreateBody(bodyDef);
  CreateLittleBody(h, g, k, d);
  k.SetAngle(b);
  k.SetID(d);
  bodyList[d] = k;
  drawnBody.push(k);
  k.SetFixedRotation(c);
  e &&
    c &&
    ((b = { x: 0, y: 0 }),
    (b.y =
      300 >= 30 * a.y
        ? (250 + 2 * createOffSet()) / drawScale
        : (350 + 2 * createOffSet()) / drawScale),
    (b.x =
      650 >= 30 * a.x
        ? (550 + 2 * createOffSet()) / drawScale
        : (750 + 2 * createOffSet()) / drawScale),
    animation.push({ id: d, startpos: b, endpos: a, count: 0, editflag: 4 }));
  f && (selectedBody = preselectBody = k);
}
function SaveNow() {
  for (var a = [], b = [], c = 0; c < wordnum; c++) {
    var d = {};
    d.x = bodyList[c].GetPosition().x;
    d.y = bodyList[c].GetPosition().y;
    a.push(d);
    b.push(bodyList[c].GetAngle());
  }
  return {
    wordsPosition: a,
    wordsAngle: b,
    wordsIfatr: [].concat(ifattract),
    wordsTagColor: [].concat(tagColor),
    wordsIffix: bodyList[0].IsFixedRotation(),
    wordsiternum: iternum,
  };
}
function SavePreEditPosition() {
  for (var a = [], b = [], c = 0; c < bodyList.length; c++) {
    var d = {};
    d.x = bodyList[c].GetPosition().x;
    d.y = bodyList[c].GetPosition().y;
    a.push(d);
    b.push(bodyList[c].GetAngle());
  }
  a = {
    prewo: [].concat(wo),
    prewe: [].concat(we),
    preTagColor: [].concat(tagColor),
    preTagFont: [].concat(tagFont),
    prewordnum: wordnum,
    prePosition: a,
    preAngle: b,
  };
  11 > preEditPoint
    ? ((preEditInfo[preEditPoint] = a),
      preEditPoint++,
      preEditInfo.length != preEditPoint &&
        preEditInfo.splice(preEditPoint, preEditInfo.length - preEditPoint))
    : (preEditInfo.splice(0, 1), (preEditInfo[preEditPoint - 1] = a));
}
function PreOrAfterEdit(a) {
  var b;
  hasselect.flag = !1;
  if (a)
    if (
      (preEditPoint == preEditInfo.length && SavePreEditPosition(),
      1 < preEditPoint)
    ) {
      iternum = 1e3;
      a = preEditInfo[preEditPoint - 2];
      if (a.prewordnum > wordnum) {
        var c = a.prewordnum - wordnum;
        var d = wordnum + c - 1;
        for (b = c; 0 < b; b--) {
          var e = a.prewo.indexOf(wo[d]);
          ChangeArrayPostion(e, d);
        }
        wordnum += c;
        for (d = 0; d < bodyList.length; d++) bodyList[d].SetID(d);
        ifstart = !1;
        for (d = 0; d < wordnum; d++)
          if (
            ((b = {
              x: bodyList[d].GetPosition().x,
              y: bodyList[d].GetPosition().y,
            }),
            (c = a.prePosition[d]),
            !(0.01 > Math.abs(b.x - c.x) && 0.01 > Math.abs(b.y - c.y)))
          ) {
            for (var f = bodyList[d].GetFixtureList(); f; f = f.m_next)
              f.SetSensor(!0);
            e = { id: d, startpos: b, endpos: c, count: 0, editflag: 2 };
            animation.push(e);
          }
      } else if (a.prewordnum < wordnum) {
        for (d = 0; d < wordnum; d++)
          if (wo[d] != a.prewo[d]) {
            e = d;
            break;
          }
        DeleteWord(!1, e);
      } else if (a.prewordnum == wordnum) {
        for (d = 0; d < wordnum; d++) {
          bodyList[d].SetAngle(a.preAngle[d]);
          b = {
            x: bodyList[d].GetPosition().x,
            y: bodyList[d].GetPosition().y,
          };
          c = a.prePosition[d];
          if (!(0.03 > Math.abs(b.x - c.x) && 0.03 > Math.abs(b.y - c.y))) {
            for (f = bodyList[d].GetFixtureList(); f; f = f.m_next)
              f.SetSensor(!0);
            e = { id: d, startpos: b, endpos: c, count: 0, editflag: 2 };
            animation.push(e);
          }
          we[d] != a.prewe[d] && ScaleWords(d, a.prewe[d]);
          tagFont[d] != a.preTagFont[d] && SetWordFont(!0, d, a.preTagFont[d]);
        }
        tagColor = [].concat(a.preTagColor);
      }
      preEditPoint--;
    } else alert("No more PreEdit information");
  else if (preEditPoint < preEditInfo.length) {
    iternum = 1e3;
    a = preEditInfo[preEditPoint];
    if (a.prewordnum < wordnum)
      for (d = a.prewo.length - 1; d >= a.prewordnum; d--)
        (e = wo.indexOf(a.prewo[d])), DeleteWord(!1, e);
    else if (a.prewordnum > wordnum) {
      c = a.prewordnum - wordnum;
      d = wordnum + c - 1;
      e = a.prewo.indexOf(wo[d]);
      ChangeArrayPostion(e, d);
      wordnum++;
      for (d = 0; d < bodyList.length; d++) bodyList[d].SetID(d);
      ifstart = !1;
      for (f = bodyList[e].GetFixtureList(); f; f = f.m_next) f.SetSensor(!0);
      b = { x: bodyList[e].GetPosition().x, y: bodyList[e].GetPosition().y };
      c = a.prePosition[e];
      (0.03 > Math.abs(b.x - c.x) && 0.03 > Math.abs(b.y - c.y)) ||
        ((e = { id: e, startpos: b, endpos: c, count: 0, editflag: 2 }),
        animation.push(e));
    } else if (a.prewordnum == wordnum) {
      for (d = 0; d < wordnum; d++) {
        bodyList[d].SetAngle(a.preAngle[d]);
        b = { x: bodyList[d].GetPosition().x, y: bodyList[d].GetPosition().y };
        c = a.prePosition[d];
        if (!(0.01 > Math.abs(b.x - c.x) && 0.01 > Math.abs(b.y - c.y))) {
          for (f = bodyList[d].GetFixtureList(); f; f = f.m_next)
            f.SetSensor(!0);
          e = { id: d, startpos: b, endpos: c, count: 0, editflag: 2 };
          animation.push(e);
        }
        we[d] != a.prewe[d] && ScaleWords(d, a.prewe[d]);
        tagFont[d] != a.preTagFont[d] && SetWordFont(!0, d, a.preTagFont[d]);
      }
      tagColor = [].concat(a.preTagColor);
    }
    preEditPoint++;
    preEditPoint == preEditInfo.length &&
      (preEditInfo.splice(preEditInfo.length - 1, 1), preEditPoint--);
  } else alert("No more AfterEdit information");
}
function ChangeArrayPostion(a, b) {
  var c = wo[b],
    d = we[b],
    e = bodyList[b],
    f = tagColor[b],
    g = tagFont[b],
    h = bodywidth[b],
    k = bodyheight[b],
    l = bodydesent[b];
  wo.splice(b, 1);
  we.splice(b, 1);
  bodyList.splice(b, 1);
  tagColor.splice(b, 1);
  tagFont.splice(b, 1);
  bodywidth.splice(b, 1);
  bodyheight.splice(b, 1);
  bodydesent.splice(b, 1);
  ifattract.splice(b, 1);
  wo.splice(a, 0, c);
  we.splice(a, 0, d);
  bodyList.splice(a, 0, e);
  tagColor.splice(a, 0, f);
  tagFont.splice(a, 0, g);
  bodywidth.splice(a, 0, h);
  bodyheight.splice(a, 0, k);
  bodydesent.splice(a, 0, l);
  ifattract.splice(a, 0, !0);
}
function SaveMovePositon() {
  AddForce = !1;
  for (var a = 0; a < wordnum; a++) {
    var b = bodyList[a].GetPosition();
    preMovePosition[a] = { x: b.x, y: b.y };
    bodyList[a].SetAngularVelocity(0);
    bodyList[a].SetLinearVelocity(new b2Vec2(0, 0));
  }
}
function LoadMovePosition(a, b) {
  for (var c = 0; c < wordnum; c++) {
    if (!b) {
      if (c == a) continue;
    } else if (-1 != multibodies.indexOf(c)) continue;
    for (var d = bodyList[c].GetFixtureList(); d; d = d.m_next) d.SetSensor(!0);
    bodyList[c].SetPosition(
      new b2Vec2(preMovePosition[c].x, preMovePosition[c].y)
    );
  }
  for (c = 0; c < wordnum; c++)
    for (d = bodyList[c].GetFixtureList(); d; d = d.m_next) d.SetSensor(!1);
  for (c = 0; c < wordnum; c++)
    (d = bodyList[c].GetPosition()),
      (preMovePosition[c] = { x: d.x, y: d.y }),
      bodyList[c].SetAngularVelocity(0),
      bodyList[c].SetLinearVelocity(new b2Vec2(0, 0));
}
function ReLayout() {
  SavePreEditPosition();
  ifedit = hasselect.flag = !1;
  for (var a = 0; a < wordnum; a++) prePosition[a] = bodyList[a].GetPosition();
  a = [].concat(tagColor);
  DestroyBody(!1);
  tagColor = a;
  createObject(void 0, void 0);
  isFirstContrust = !0;
}
function DestroyBody(a) {
  if (a) {
    for (a = 0; a < bodyList.length; a++) world.DestroyBody(bodyList[a]);
    bodyList = [];
    bodywidth = [];
    bodyheight = [];
    wordsNeighbor = [];
    prePosition = [];
    ifattract = [];
    tagColor = [];
  } else
    for (a = 0; a < wordnum; a++)
      world.DestroyBody(bodyList[a]),
        (bodyList[a] = void 0),
        (bodywidth[a] = void 0),
        (bodyheight[a] = void 0),
        (wordsNeighbor[a] = void 0),
        (ifattract[a] = void 0),
        (tagColor[a] = void 0);
  havedecent = ifstart = !1;
  drawnBody = [];
}
function ReSortWoandWe(a, b) {
  for (var c = 0; c < wordnum; c++)
    for (var d = 0; d < wordnum - c - 1; d++)
      if (we[d] < we[d + 1]) {
        var e = we[d],
          f = wo[d],
          g = a[d],
          h = b[d],
          k = tagColor[d];
        we[d] = we[d + 1];
        wo[d] = wo[d + 1];
        a[d] = a[d + 1];
        b[d] = b[d + 1];
        tagColor[d] = tagColor[d + 1];
        we[d + 1] = e;
        wo[d + 1] = f;
        a[d + 1] = g;
        b[d + 1] = h;
        tagColor[d + 1] = k;
      }
}
String.prototype.isInChinese = function () {
  return this.length != this.replace(/[^\x00-\xff]/g, "**").length;
};
function getElementPosition() {
  var a = $(window).scrollTop();
  var b = $("#canvas").offset().left;
  a = $("#canvas").offset().top - a;
  return { x: b, y: a };
}
$(document).ready(function () {
  $("#canvas").bind("contextmenu", function (a) {
    return !1;
  });
});
function measureTextH_W(a, b, c, d, e, f, g, h) {
  a.clearRect(0, 0, 450, 400);
  a.save();
  a.translate(b, f + 10);
  a.font = f + "px " + g;
  a.fillStyle = "#000000";
  a.fillText(h, 0, 0);
  g = a.measureText(h).width;
  a.restore();
  b = a.getImageData(b, c, d, e).data;
  h = c = !1;
  for (var k = 0, l; !h && e; )
    for (e--, l = 0; l < d; l++)
      if (b[e * d * 4 + 4 * l + 3]) {
        h = e;
        break;
      }
  for (; e; )
    for (e--, l = 0; l < d; l++)
      if (b[e * d * 4 + 4 * l + 3]) {
        e > f + 10 && k++;
        c = e;
        break;
      }
  return c != e
    ? (a.clearRect(0, 0, 150, 150), { height: h - c, width: g, desent: k })
    : 0;
}
function CorrectCenter(a, b) {
  var c =
      ((maxfontsize - minfontsize) * we[a] + minfontsize) / 2 -
      bodyheight[a] +
      bodydesent[a],
    d = bodyList[a].GetAngle(),
    e = b.x,
    c = b.y + c / drawScale;
  if (0 != d)
    var f = (e - b.x) * Math.cos(d) - (c - b.y) * Math.sin(d) + b.x,
      c = (e - b.x) * Math.sin(d) + (c - b.y) * Math.cos(d) + b.y,
      e = f;
  return new b2Vec2(e, c);
}
Array.prototype.remove = function (a) {
  a = this.indexOf(a);
  -1 < a && this.splice(a, 1);
};
