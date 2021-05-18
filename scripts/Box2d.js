var Box2D = {};
(function (n, x) {
  function t() {}
  !(Object.prototype.defineProperty instanceof Function) &&
    Object.prototype.__defineGetter__ instanceof Function &&
    Object.prototype.__defineSetter__ instanceof Function &&
    (Object.defineProperty = function (c, h, l) {
      l.get instanceof Function && c.__defineGetter__(h, l.get);
      l.set instanceof Function && c.__defineSetter__(h, l.set);
    });
  n.inherit = function (c, h) {
    t.prototype = h.prototype;
    c.prototype = new t();
    c.prototype.constructor = c;
  };
  n.generateCallback = function (c, h) {
    return function () {
      h.apply(c, arguments);
    };
  };
  n.NVector = function (c) {
    c === x && (c = 0);
    for (var h = Array(c || 0), l = 0; l < c; ++l) h[l] = 0;
    return h;
  };
  n.is = function (c, h) {
    return null === c
      ? !1
      : (h instanceof Function && c instanceof h) ||
        (c.constructor.__implements != x && c.constructor.__implements[h])
      ? !0
      : !1;
  };
  n.parseUInt = function (c) {
    return Math.abs(parseInt(c));
  };
})(Box2D);
var Vector = Array,
  Vector_a2j_Number = Box2D.NVector;
"undefined" === typeof Box2D && (Box2D = {});
"undefined" === typeof Box2D.Collision && (Box2D.Collision = {});
"undefined" === typeof Box2D.Collision.Shapes && (Box2D.Collision.Shapes = {});
"undefined" === typeof Box2D.Common && (Box2D.Common = {});
"undefined" === typeof Box2D.Common.Math && (Box2D.Common.Math = {});
"undefined" === typeof Box2D.Dynamics && (Box2D.Dynamics = {});
"undefined" === typeof Box2D.Dynamics.Contacts &&
  (Box2D.Dynamics.Contacts = {});
"undefined" === typeof Box2D.Dynamics.Controllers &&
  (Box2D.Dynamics.Controllers = {});
"undefined" === typeof Box2D.Dynamics.Joints && (Box2D.Dynamics.Joints = {});
(function () {
  function n() {
    n.b2AABB.apply(this, arguments);
  }
  function x() {
    x.b2Bound.apply(this, arguments);
  }
  function t() {
    t.b2BoundValues.apply(this, arguments);
    this.constructor === t && this.b2BoundValues.apply(this, arguments);
  }
  function c() {
    c.b2Collision.apply(this, arguments);
  }
  function h() {
    h.b2ContactID.apply(this, arguments);
    this.constructor === h && this.b2ContactID.apply(this, arguments);
  }
  function l() {
    l.b2ContactPoint.apply(this, arguments);
  }
  function A() {
    A.b2Distance.apply(this, arguments);
  }
  function e() {
    e.b2DistanceInput.apply(this, arguments);
  }
  function w() {
    w.b2DistanceOutput.apply(this, arguments);
  }
  function G() {
    G.b2DistanceProxy.apply(this, arguments);
  }
  function P() {
    P.b2DynamicTree.apply(this, arguments);
    this.constructor === P && this.b2DynamicTree.apply(this, arguments);
  }
  function H() {
    H.b2DynamicTreeBroadPhase.apply(this, arguments);
  }
  function L() {
    L.b2DynamicTreeNode.apply(this, arguments);
  }
  function y() {
    y.b2DynamicTreePair.apply(this, arguments);
  }
  function Q() {
    Q.b2Manifold.apply(this, arguments);
    this.constructor === Q && this.b2Manifold.apply(this, arguments);
  }
  function u() {
    u.b2ManifoldPoint.apply(this, arguments);
    this.constructor === u && this.b2ManifoldPoint.apply(this, arguments);
  }
  function g() {
    g.b2Point.apply(this, arguments);
  }
  function C() {
    C.b2RayCastInput.apply(this, arguments);
    this.constructor === C && this.b2RayCastInput.apply(this, arguments);
  }
  function E() {
    E.b2RayCastOutput.apply(this, arguments);
  }
  function M() {
    M.b2Segment.apply(this, arguments);
  }
  function K() {
    K.b2SeparationFunction.apply(this, arguments);
  }
  function N() {
    N.b2Simplex.apply(this, arguments);
    this.constructor === N && this.b2Simplex.apply(this, arguments);
  }
  function D() {
    D.b2SimplexCache.apply(this, arguments);
  }
  function R() {
    R.b2SimplexVertex.apply(this, arguments);
  }
  function F() {
    F.b2TimeOfImpact.apply(this, arguments);
  }
  function I() {
    I.b2TOIInput.apply(this, arguments);
  }
  function S() {
    S.b2WorldManifold.apply(this, arguments);
    this.constructor === S && this.b2WorldManifold.apply(this, arguments);
  }
  function O() {
    O.ClipVertex.apply(this, arguments);
  }
  function a() {
    a.Features.apply(this, arguments);
  }
  function d() {
    d.b2CircleShape.apply(this, arguments);
    this.constructor === d && this.b2CircleShape.apply(this, arguments);
  }
  function z() {
    z.b2EdgeChainDef.apply(this, arguments);
    this.constructor === z && this.b2EdgeChainDef.apply(this, arguments);
  }
  function f() {
    f.b2EdgeShape.apply(this, arguments);
    this.constructor === f && this.b2EdgeShape.apply(this, arguments);
  }
  function r() {
    r.b2MassData.apply(this, arguments);
  }
  function v() {
    v.b2PolygonShape.apply(this, arguments);
    this.constructor === v && this.b2PolygonShape.apply(this, arguments);
  }
  function J() {
    J.b2Shape.apply(this, arguments);
    this.constructor === J && this.b2Shape.apply(this, arguments);
  }
  function b() {
    b.b2Color.apply(this, arguments);
    this.constructor === b && this.b2Color.apply(this, arguments);
  }
  function k() {
    k.b2Settings.apply(this, arguments);
  }
  function q() {
    q.b2Mat22.apply(this, arguments);
    this.constructor === q && this.b2Mat22.apply(this, arguments);
  }
  function m() {
    m.b2Mat33.apply(this, arguments);
    this.constructor === m && this.b2Mat33.apply(this, arguments);
  }
  function p() {
    p.b2Math.apply(this, arguments);
  }
  function B() {
    B.b2Sweep.apply(this, arguments);
  }
  function T() {
    T.b2Transform.apply(this, arguments);
    this.constructor === T && this.b2Transform.apply(this, arguments);
  }
  function U() {
    U.b2Vec2.apply(this, arguments);
    this.constructor === U && this.b2Vec2.apply(this, arguments);
  }
  function V() {
    V.b2Vec3.apply(this, arguments);
    this.constructor === V && this.b2Vec3.apply(this, arguments);
  }
  function W() {
    W.b2Body.apply(this, arguments);
    this.constructor === W && this.b2Body.apply(this, arguments);
  }
  function X() {
    X.b2BodyDef.apply(this, arguments);
    this.constructor === X && this.b2BodyDef.apply(this, arguments);
  }
  function Ea() {
    Ea.b2ContactFilter.apply(this, arguments);
  }
  function Fa() {
    Fa.b2ContactImpulse.apply(this, arguments);
  }
  function Ga() {
    Ga.b2ContactListener.apply(this, arguments);
  }
  function Y() {
    Y.b2ContactManager.apply(this, arguments);
    this.constructor === Y && this.b2ContactManager.apply(this, arguments);
  }
  function Z() {
    Z.b2DebugDraw.apply(this, arguments);
    this.constructor === Z && this.b2DebugDraw.apply(this, arguments);
  }
  function Ha() {
    Ha.b2DestructionListener.apply(this, arguments);
  }
  function Ia() {
    Ia.b2FilterData.apply(this, arguments);
  }
  function aa() {
    aa.b2Fixture.apply(this, arguments);
    this.constructor === aa && this.b2Fixture.apply(this, arguments);
  }
  function ba() {
    ba.b2FixtureDef.apply(this, arguments);
    this.constructor === ba && this.b2FixtureDef.apply(this, arguments);
  }
  function ca() {
    ca.b2Island.apply(this, arguments);
    this.constructor === ca && this.b2Island.apply(this, arguments);
  }
  function Ja() {
    Ja.b2TimeStep.apply(this, arguments);
  }
  function da() {
    da.b2World.apply(this, arguments);
    this.constructor === da && this.b2World.apply(this, arguments);
  }
  function Ka() {
    Ka.b2CircleContact.apply(this, arguments);
  }
  function ea() {
    ea.b2Contact.apply(this, arguments);
    this.constructor === ea && this.b2Contact.apply(this, arguments);
  }
  function fa() {
    fa.b2ContactConstraint.apply(this, arguments);
    this.constructor === fa && this.b2ContactConstraint.apply(this, arguments);
  }
  function La() {
    La.b2ContactConstraintPoint.apply(this, arguments);
  }
  function Ma() {
    Ma.b2ContactEdge.apply(this, arguments);
  }
  function ga() {
    ga.b2ContactFactory.apply(this, arguments);
    this.constructor === ga && this.b2ContactFactory.apply(this, arguments);
  }
  function Na() {
    Na.b2ContactRegister.apply(this, arguments);
  }
  function Oa() {
    Oa.b2ContactResult.apply(this, arguments);
  }
  function ha() {
    ha.b2ContactSolver.apply(this, arguments);
    this.constructor === ha && this.b2ContactSolver.apply(this, arguments);
  }
  function Pa() {
    Pa.b2EdgeAndCircleContact.apply(this, arguments);
  }
  function ia() {
    ia.b2NullContact.apply(this, arguments);
    this.constructor === ia && this.b2NullContact.apply(this, arguments);
  }
  function Qa() {
    Qa.b2PolyAndCircleContact.apply(this, arguments);
  }
  function Ra() {
    Ra.b2PolyAndEdgeContact.apply(this, arguments);
  }
  function Sa() {
    Sa.b2PolygonContact.apply(this, arguments);
  }
  function ja() {
    ja.b2PositionSolverManifold.apply(this, arguments);
    this.constructor === ja &&
      this.b2PositionSolverManifold.apply(this, arguments);
  }
  function Ta() {
    Ta.b2BuoyancyController.apply(this, arguments);
  }
  function Ua() {
    Ua.b2ConstantAccelController.apply(this, arguments);
  }
  function Va() {
    Va.b2ConstantForceController.apply(this, arguments);
  }
  function Wa() {
    Wa.b2Controller.apply(this, arguments);
  }
  function Xa() {
    Xa.b2ControllerEdge.apply(this, arguments);
  }
  function Ya() {
    Ya.b2GravityController.apply(this, arguments);
  }
  function Za() {
    Za.b2TensorDampingController.apply(this, arguments);
  }
  function ka() {
    ka.b2DistanceJoint.apply(this, arguments);
    this.constructor === ka && this.b2DistanceJoint.apply(this, arguments);
  }
  function la() {
    la.b2DistanceJointDef.apply(this, arguments);
    this.constructor === la && this.b2DistanceJointDef.apply(this, arguments);
  }
  function ma() {
    ma.b2FrictionJoint.apply(this, arguments);
    this.constructor === ma && this.b2FrictionJoint.apply(this, arguments);
  }
  function na() {
    na.b2FrictionJointDef.apply(this, arguments);
    this.constructor === na && this.b2FrictionJointDef.apply(this, arguments);
  }
  function oa() {
    oa.b2GearJoint.apply(this, arguments);
    this.constructor === oa && this.b2GearJoint.apply(this, arguments);
  }
  function pa() {
    pa.b2GearJointDef.apply(this, arguments);
    this.constructor === pa && this.b2GearJointDef.apply(this, arguments);
  }
  function $a() {
    $a.b2Jacobian.apply(this, arguments);
  }
  function qa() {
    qa.b2Joint.apply(this, arguments);
    this.constructor === qa && this.b2Joint.apply(this, arguments);
  }
  function ra() {
    ra.b2JointDef.apply(this, arguments);
    this.constructor === ra && this.b2JointDef.apply(this, arguments);
  }
  function ab() {
    ab.b2JointEdge.apply(this, arguments);
  }
  function sa() {
    sa.b2LineJoint.apply(this, arguments);
    this.constructor === sa && this.b2LineJoint.apply(this, arguments);
  }
  function ta() {
    ta.b2LineJointDef.apply(this, arguments);
    this.constructor === ta && this.b2LineJointDef.apply(this, arguments);
  }
  function ua() {
    ua.b2MouseJoint.apply(this, arguments);
    this.constructor === ua && this.b2MouseJoint.apply(this, arguments);
  }
  function va() {
    va.b2MouseJointDef.apply(this, arguments);
    this.constructor === va && this.b2MouseJointDef.apply(this, arguments);
  }
  function wa() {
    wa.b2PrismaticJoint.apply(this, arguments);
    this.constructor === wa && this.b2PrismaticJoint.apply(this, arguments);
  }
  function xa() {
    xa.b2PrismaticJointDef.apply(this, arguments);
    this.constructor === xa && this.b2PrismaticJointDef.apply(this, arguments);
  }
  function ya() {
    ya.b2PulleyJoint.apply(this, arguments);
    this.constructor === ya && this.b2PulleyJoint.apply(this, arguments);
  }
  function za() {
    za.b2PulleyJointDef.apply(this, arguments);
    this.constructor === za && this.b2PulleyJointDef.apply(this, arguments);
  }
  function Aa() {
    Aa.b2RevoluteJoint.apply(this, arguments);
    this.constructor === Aa && this.b2RevoluteJoint.apply(this, arguments);
  }
  function Ba() {
    Ba.b2RevoluteJointDef.apply(this, arguments);
    this.constructor === Ba && this.b2RevoluteJointDef.apply(this, arguments);
  }
  function Ca() {
    Ca.b2WeldJoint.apply(this, arguments);
    this.constructor === Ca && this.b2WeldJoint.apply(this, arguments);
  }
  function Da() {
    Da.b2WeldJointDef.apply(this, arguments);
    this.constructor === Da && this.b2WeldJointDef.apply(this, arguments);
  }
  Box2D.Collision.IBroadPhase = "Box2D.Collision.IBroadPhase";
  Box2D.Collision.b2AABB = n;
  Box2D.Collision.b2Bound = x;
  Box2D.Collision.b2BoundValues = t;
  Box2D.Collision.b2Collision = c;
  Box2D.Collision.b2ContactID = h;
  Box2D.Collision.b2ContactPoint = l;
  Box2D.Collision.b2Distance = A;
  Box2D.Collision.b2DistanceInput = e;
  Box2D.Collision.b2DistanceOutput = w;
  Box2D.Collision.b2DistanceProxy = G;
  Box2D.Collision.b2DynamicTree = P;
  Box2D.Collision.b2DynamicTreeBroadPhase = H;
  Box2D.Collision.b2DynamicTreeNode = L;
  Box2D.Collision.b2DynamicTreePair = y;
  Box2D.Collision.b2Manifold = Q;
  Box2D.Collision.b2ManifoldPoint = u;
  Box2D.Collision.b2Point = g;
  Box2D.Collision.b2RayCastInput = C;
  Box2D.Collision.b2RayCastOutput = E;
  Box2D.Collision.b2Segment = M;
  Box2D.Collision.b2SeparationFunction = K;
  Box2D.Collision.b2Simplex = N;
  Box2D.Collision.b2SimplexCache = D;
  Box2D.Collision.b2SimplexVertex = R;
  Box2D.Collision.b2TimeOfImpact = F;
  Box2D.Collision.b2TOIInput = I;
  Box2D.Collision.b2WorldManifold = S;
  Box2D.Collision.ClipVertex = O;
  Box2D.Collision.Features = a;
  Box2D.Collision.Shapes.b2CircleShape = d;
  Box2D.Collision.Shapes.b2EdgeChainDef = z;
  Box2D.Collision.Shapes.b2EdgeShape = f;
  Box2D.Collision.Shapes.b2MassData = r;
  Box2D.Collision.Shapes.b2PolygonShape = v;
  Box2D.Collision.Shapes.b2Shape = J;
  Box2D.Common.b2internal = "Box2D.Common.b2internal";
  Box2D.Common.b2Color = b;
  Box2D.Common.b2Settings = k;
  Box2D.Common.Math.b2Mat22 = q;
  Box2D.Common.Math.b2Mat33 = m;
  Box2D.Common.Math.b2Math = p;
  Box2D.Common.Math.b2Sweep = B;
  Box2D.Common.Math.b2Transform = T;
  Box2D.Common.Math.b2Vec2 = U;
  Box2D.Common.Math.b2Vec3 = V;
  Box2D.Dynamics.b2Body = W;
  Box2D.Dynamics.b2BodyDef = X;
  Box2D.Dynamics.b2ContactFilter = Ea;
  Box2D.Dynamics.b2ContactImpulse = Fa;
  Box2D.Dynamics.b2ContactListener = Ga;
  Box2D.Dynamics.b2ContactManager = Y;
  Box2D.Dynamics.b2DebugDraw = Z;
  Box2D.Dynamics.b2DestructionListener = Ha;
  Box2D.Dynamics.b2FilterData = Ia;
  Box2D.Dynamics.b2Fixture = aa;
  Box2D.Dynamics.b2FixtureDef = ba;
  Box2D.Dynamics.b2Island = ca;
  Box2D.Dynamics.b2TimeStep = Ja;
  Box2D.Dynamics.b2World = da;
  Box2D.Dynamics.Contacts.b2CircleContact = Ka;
  Box2D.Dynamics.Contacts.b2Contact = ea;
  Box2D.Dynamics.Contacts.b2ContactConstraint = fa;
  Box2D.Dynamics.Contacts.b2ContactConstraintPoint = La;
  Box2D.Dynamics.Contacts.b2ContactEdge = Ma;
  Box2D.Dynamics.Contacts.b2ContactFactory = ga;
  Box2D.Dynamics.Contacts.b2ContactRegister = Na;
  Box2D.Dynamics.Contacts.b2ContactResult = Oa;
  Box2D.Dynamics.Contacts.b2ContactSolver = ha;
  Box2D.Dynamics.Contacts.b2EdgeAndCircleContact = Pa;
  Box2D.Dynamics.Contacts.b2NullContact = ia;
  Box2D.Dynamics.Contacts.b2PolyAndCircleContact = Qa;
  Box2D.Dynamics.Contacts.b2PolyAndEdgeContact = Ra;
  Box2D.Dynamics.Contacts.b2PolygonContact = Sa;
  Box2D.Dynamics.Contacts.b2PositionSolverManifold = ja;
  Box2D.Dynamics.Controllers.b2BuoyancyController = Ta;
  Box2D.Dynamics.Controllers.b2ConstantAccelController = Ua;
  Box2D.Dynamics.Controllers.b2ConstantForceController = Va;
  Box2D.Dynamics.Controllers.b2Controller = Wa;
  Box2D.Dynamics.Controllers.b2ControllerEdge = Xa;
  Box2D.Dynamics.Controllers.b2GravityController = Ya;
  Box2D.Dynamics.Controllers.b2TensorDampingController = Za;
  Box2D.Dynamics.Joints.b2DistanceJoint = ka;
  Box2D.Dynamics.Joints.b2DistanceJointDef = la;
  Box2D.Dynamics.Joints.b2FrictionJoint = ma;
  Box2D.Dynamics.Joints.b2FrictionJointDef = na;
  Box2D.Dynamics.Joints.b2GearJoint = oa;
  Box2D.Dynamics.Joints.b2GearJointDef = pa;
  Box2D.Dynamics.Joints.b2Jacobian = $a;
  Box2D.Dynamics.Joints.b2Joint = qa;
  Box2D.Dynamics.Joints.b2JointDef = ra;
  Box2D.Dynamics.Joints.b2JointEdge = ab;
  Box2D.Dynamics.Joints.b2LineJoint = sa;
  Box2D.Dynamics.Joints.b2LineJointDef = ta;
  Box2D.Dynamics.Joints.b2MouseJoint = ua;
  Box2D.Dynamics.Joints.b2MouseJointDef = va;
  Box2D.Dynamics.Joints.b2PrismaticJoint = wa;
  Box2D.Dynamics.Joints.b2PrismaticJointDef = xa;
  Box2D.Dynamics.Joints.b2PulleyJoint = ya;
  Box2D.Dynamics.Joints.b2PulleyJointDef = za;
  Box2D.Dynamics.Joints.b2RevoluteJoint = Aa;
  Box2D.Dynamics.Joints.b2RevoluteJointDef = Ba;
  Box2D.Dynamics.Joints.b2WeldJoint = Ca;
  Box2D.Dynamics.Joints.b2WeldJointDef = Da;
})();
Box2D.postDefs = [];
(function () {
  var n = Box2D.Collision.Shapes.b2CircleShape,
    x = Box2D.Collision.Shapes.b2PolygonShape,
    t = Box2D.Collision.Shapes.b2Shape,
    c = Box2D.Common.b2Settings,
    h = Box2D.Common.Math.b2Math,
    l = Box2D.Common.Math.b2Sweep,
    A = Box2D.Common.Math.b2Transform,
    e = Box2D.Common.Math.b2Vec2,
    w = Box2D.Collision.b2AABB,
    G = Box2D.Collision.b2Bound,
    P = Box2D.Collision.b2BoundValues,
    H = Box2D.Collision.b2Collision,
    L = Box2D.Collision.b2ContactID,
    y = Box2D.Collision.b2ContactPoint,
    Q = Box2D.Collision.b2Distance,
    u = Box2D.Collision.b2DistanceInput,
    g = Box2D.Collision.b2DistanceOutput,
    C = Box2D.Collision.b2DistanceProxy,
    E = Box2D.Collision.b2DynamicTree,
    M = Box2D.Collision.b2DynamicTreeBroadPhase,
    K = Box2D.Collision.b2DynamicTreeNode,
    N = Box2D.Collision.b2DynamicTreePair,
    D = Box2D.Collision.b2Manifold,
    R = Box2D.Collision.b2ManifoldPoint,
    F = Box2D.Collision.b2Point,
    I = Box2D.Collision.b2RayCastInput,
    S = Box2D.Collision.b2RayCastOutput,
    O = Box2D.Collision.b2Segment,
    a = Box2D.Collision.b2SeparationFunction,
    d = Box2D.Collision.b2Simplex,
    z = Box2D.Collision.b2SimplexCache,
    f = Box2D.Collision.b2SimplexVertex,
    r = Box2D.Collision.b2TimeOfImpact,
    v = Box2D.Collision.b2TOIInput,
    J = Box2D.Collision.b2WorldManifold,
    b = Box2D.Collision.ClipVertex,
    k = Box2D.Collision.Features,
    q = Box2D.Collision.IBroadPhase;
  w.b2AABB = function () {
    this.lowerBound = new e();
    this.upperBound = new e();
  };
  w.prototype.IsValid = function () {
    var b = this.upperBound.y - this.lowerBound.y;
    return (
      0 <= this.upperBound.x - this.lowerBound.x &&
      0 <= b &&
      this.lowerBound.IsValid() &&
      this.upperBound.IsValid()
    );
  };
  w.prototype.GetCenter = function () {
    return new e(
      (this.lowerBound.x + this.upperBound.x) / 2,
      (this.lowerBound.y + this.upperBound.y) / 2
    );
  };
  w.prototype.GetExtents = function () {
    return new e(
      (this.upperBound.x - this.lowerBound.x) / 2,
      (this.upperBound.y - this.lowerBound.y) / 2
    );
  };
  w.prototype.Contains = function (b) {
    var m;
    return (m =
      (m =
        (m =
          (m = this.lowerBound.x <= b.lowerBound.x) &&
          this.lowerBound.y <= b.lowerBound.y) &&
        b.upperBound.x <= this.upperBound.x) &&
      b.upperBound.y <= this.upperBound.y);
  };
  w.prototype.RayCast = function (b, a) {
    var m = -Number.MAX_VALUE,
      k = Number.MAX_VALUE,
      p = a.p1.x,
      q = a.p1.y,
      d = a.p2.x - a.p1.x,
      f = a.p2.y - a.p1.y,
      r = Math.abs(f),
      v = b.normal;
    if (Math.abs(d) < Number.MIN_VALUE) {
      if (p < this.lowerBound.x || this.upperBound.x < p) return !1;
    } else {
      var z = 1 / d;
      d = (this.lowerBound.x - p) * z;
      p = (this.upperBound.x - p) * z;
      z = -1;
      d > p && ((z = d), (d = p), (p = z), (z = 1));
      d > m && ((v.x = z), (v.y = 0), (m = d));
      k = Math.min(k, p);
      if (m > k) return !1;
    }
    if (r < Number.MIN_VALUE) {
      if (q < this.lowerBound.y || this.upperBound.y < q) return !1;
    } else if (
      ((z = 1 / f),
      (d = (this.lowerBound.y - q) * z),
      (p = (this.upperBound.y - q) * z),
      (z = -1),
      d > p && ((z = d), (d = p), (p = z), (z = 1)),
      d > m && ((v.y = z), (v.x = 0), (m = d)),
      (k = Math.min(k, p)),
      m > k)
    )
      return !1;
    b.fraction = m;
    return !0;
  };
  w.prototype.TestOverlap = function (b) {
    var m = b.lowerBound.y - this.upperBound.y,
      a = this.lowerBound.y - b.upperBound.y;
    return 0 < b.lowerBound.x - this.upperBound.x ||
      0 < m ||
      0 < this.lowerBound.x - b.upperBound.x ||
      0 < a
      ? !1
      : !0;
  };
  w.Combine = function (b, a) {
    var m = new w();
    m.Combine(b, a);
    return m;
  };
  w.prototype.Combine = function (b, a) {
    this.lowerBound.x = Math.min(b.lowerBound.x, a.lowerBound.x);
    this.lowerBound.y = Math.min(b.lowerBound.y, a.lowerBound.y);
    this.upperBound.x = Math.max(b.upperBound.x, a.upperBound.x);
    this.upperBound.y = Math.max(b.upperBound.y, a.upperBound.y);
  };
  G.b2Bound = function () {};
  G.prototype.IsLower = function () {
    return 0 == (this.value & 1);
  };
  G.prototype.IsUpper = function () {
    return 1 == (this.value & 1);
  };
  G.prototype.Swap = function (b) {
    var m = this.value,
      a = this.proxy,
      k = this.stabbingCount;
    this.value = b.value;
    this.proxy = b.proxy;
    this.stabbingCount = b.stabbingCount;
    b.value = m;
    b.proxy = a;
    b.stabbingCount = k;
  };
  P.b2BoundValues = function () {};
  P.prototype.b2BoundValues = function () {
    this.lowerValues = new Vector_a2j_Number();
    this.lowerValues[0] = 0;
    this.lowerValues[1] = 0;
    this.upperValues = new Vector_a2j_Number();
    this.upperValues[0] = 0;
    this.upperValues[1] = 0;
  };
  H.b2Collision = function () {};
  H.ClipSegmentToLine = function (b, a, k, q) {
    void 0 === q && (q = 0);
    var m = 0;
    var p = a[0];
    var B = p.v;
    p = a[1];
    var d = p.v,
      f = k.x * B.x + k.y * B.y - q;
    p = k.x * d.x + k.y * d.y - q;
    0 >= f && b[m++].Set(a[0]);
    0 >= p && b[m++].Set(a[1]);
    0 > f * p &&
      ((k = f / (f - p)),
      (p = b[m]),
      (p = p.v),
      (p.x = B.x + k * (d.x - B.x)),
      (p.y = B.y + k * (d.y - B.y)),
      (p = b[m]),
      (p.id = (0 < f ? a[0] : a[1]).id),
      ++m);
    return m;
  };
  H.EdgeSeparation = function (b, a, k, q, d) {
    void 0 === k && (k = 0);
    parseInt(b.m_vertexCount);
    var m = b.m_vertices,
      p = b.m_normals;
    b = parseInt(q.m_vertexCount);
    q = q.m_vertices;
    var B = a.R;
    var f = p[k];
    var p = B.col1.x * f.x + B.col2.x * f.y,
      r = B.col1.y * f.x + B.col2.y * f.y;
    B = d.R;
    var z = B.col1.x * p + B.col1.y * r;
    B = B.col2.x * p + B.col2.y * r;
    for (var v = 0, T = Number.MAX_VALUE, g = 0; g < b; ++g)
      (f = q[g]), (f = f.x * z + f.y * B), f < T && ((T = f), (v = g));
    f = m[k];
    B = a.R;
    k = a.position.x + (B.col1.x * f.x + B.col2.x * f.y);
    a = a.position.y + (B.col1.y * f.x + B.col2.y * f.y);
    f = q[v];
    B = d.R;
    return (
      (d.position.x + (B.col1.x * f.x + B.col2.x * f.y) - k) * p +
      (d.position.y + (B.col1.y * f.x + B.col2.y * f.y) - a) * r
    );
  };
  H.FindMaxSeparation = function (b, a, k, q, d) {
    var m = parseInt(a.m_vertexCount),
      p = a.m_normals;
    var B = d.R;
    var f = q.m_centroid;
    var r = d.position.x + (B.col1.x * f.x + B.col2.x * f.y),
      z = d.position.y + (B.col1.y * f.x + B.col2.y * f.y);
    B = k.R;
    f = a.m_centroid;
    r -= k.position.x + (B.col1.x * f.x + B.col2.x * f.y);
    z -= k.position.y + (B.col1.y * f.x + B.col2.y * f.y);
    B = r * k.R.col1.x + z * k.R.col1.y;
    for (
      var z = r * k.R.col2.x + z * k.R.col2.y,
        r = 0,
        v = -Number.MAX_VALUE,
        T = 0;
      T < m;
      ++T
    )
      (f = p[T]), (f = f.x * B + f.y * z), f > v && ((v = f), (r = T));
    p = H.EdgeSeparation(a, k, r, q, d);
    v = parseInt(0 <= r - 1 ? r - 1 : m - 1);
    B = H.EdgeSeparation(a, k, v, q, d);
    T = parseInt(r + 1 < m ? r + 1 : 0);
    z = H.EdgeSeparation(a, k, T, q, d);
    if (B > p && B > z) f = -1;
    else if (z > p) (f = 1), (v = T), (B = z);
    else return (b[0] = r), p;
    for (;;)
      if (
        ((r = -1 == f ? (0 <= v - 1 ? v - 1 : m - 1) : v + 1 < m ? v + 1 : 0),
        (p = H.EdgeSeparation(a, k, r, q, d)),
        p > B)
      )
        (v = r), (B = p);
      else break;
    b[0] = v;
    return B;
  };
  H.FindIncidentEdge = function (b, a, k, q, f, d) {
    void 0 === q && (q = 0);
    parseInt(a.m_vertexCount);
    var m = a.m_normals,
      p = parseInt(f.m_vertexCount);
    a = f.m_vertices;
    f = f.m_normals;
    var B = k.R;
    k = m[q];
    var m = B.col1.x * k.x + B.col2.x * k.y,
      r = B.col1.y * k.x + B.col2.y * k.y;
    B = d.R;
    k = B.col1.x * m + B.col1.y * r;
    r = B.col2.x * m + B.col2.y * r;
    m = k;
    B = 0;
    for (var v = Number.MAX_VALUE, z = 0; z < p; ++z)
      (k = f[z]), (k = m * k.x + r * k.y), k < v && ((v = k), (B = z));
    f = parseInt(B);
    m = parseInt(f + 1 < p ? f + 1 : 0);
    p = b[0];
    k = a[f];
    B = d.R;
    p.v.x = d.position.x + (B.col1.x * k.x + B.col2.x * k.y);
    p.v.y = d.position.y + (B.col1.y * k.x + B.col2.y * k.y);
    p.id.features.referenceEdge = q;
    p.id.features.incidentEdge = f;
    p.id.features.incidentVertex = 0;
    p = b[1];
    k = a[m];
    B = d.R;
    p.v.x = d.position.x + (B.col1.x * k.x + B.col2.x * k.y);
    p.v.y = d.position.y + (B.col1.y * k.x + B.col2.y * k.y);
    p.id.features.referenceEdge = q;
    p.id.features.incidentEdge = m;
    p.id.features.incidentVertex = 1;
  };
  H.MakeClipPointVector = function () {
    var m = new Vector(2);
    m[0] = new b();
    m[1] = new b();
    return m;
  };
  H.CollidePolygons = function (b, a, k, q, f) {
    b.m_pointCount = 0;
    var m = a.m_radius + q.m_radius;
    H.s_edgeAO[0] = 0;
    var p = H.FindMaxSeparation(H.s_edgeAO, a, k, q, f);
    var d = H.s_edgeAO[0];
    if (!(p > m)) {
      H.s_edgeBO[0] = 0;
      var B = H.FindMaxSeparation(H.s_edgeBO, q, f, a, k);
      var r = H.s_edgeBO[0];
      if (!(B > m)) {
        B > 0.98 * p + 0.001
          ? ((p = q),
            (q = a),
            (a = f),
            (f = r),
            (b.m_type = D.e_faceB),
            (d = 1))
          : ((p = a),
            (a = k),
            (k = f),
            (f = d),
            (b.m_type = D.e_faceA),
            (d = 0));
        r = H.s_incidentEdge;
        H.FindIncidentEdge(r, p, a, f, q, k);
        B = parseInt(p.m_vertexCount);
        p = p.m_vertices;
        q = p[f];
        var z = f + 1 < B ? p[parseInt(f + 1)] : p[0];
        p = H.s_localTangent;
        p.Set(z.x - q.x, z.y - q.y);
        p.Normalize();
        B = H.s_localNormal;
        B.x = p.y;
        B.y = -p.x;
        var v = H.s_planePoint;
        v.Set(0.5 * (q.x + z.x), 0.5 * (q.y + z.y));
        var g = H.s_tangent;
        f = a.R;
        g.x = f.col1.x * p.x + f.col2.x * p.y;
        g.y = f.col1.y * p.x + f.col2.y * p.y;
        var T = H.s_tangent2;
        T.x = -g.x;
        T.y = -g.y;
        p = H.s_normal;
        p.x = g.y;
        p.y = -g.x;
        var e = H.s_v11,
          J = H.s_v12;
        e.x = a.position.x + (f.col1.x * q.x + f.col2.x * q.y);
        e.y = a.position.y + (f.col1.y * q.x + f.col2.y * q.y);
        J.x = a.position.x + (f.col1.x * z.x + f.col2.x * z.y);
        J.y = a.position.y + (f.col1.y * z.x + f.col2.y * z.y);
        a = p.x * e.x + p.y * e.y;
        f = g.x * J.x + g.y * J.y + m;
        z = H.s_clipPoints1;
        q = H.s_clipPoints2;
        r = H.ClipSegmentToLine(z, r, T, -g.x * e.x - g.y * e.y + m);
        if (!(2 > r || ((r = H.ClipSegmentToLine(q, z, g, f)), 2 > r))) {
          b.m_localPlaneNormal.SetV(B);
          b.m_localPoint.SetV(v);
          for (v = B = 0; v < c.b2_maxManifoldPoints; ++v)
            (r = q[v]),
              p.x * r.v.x + p.y * r.v.y - a <= m &&
                ((g = b.m_points[B]),
                (f = k.R),
                (T = r.v.x - k.position.x),
                (e = r.v.y - k.position.y),
                (g.m_localPoint.x = T * f.col1.x + e * f.col1.y),
                (g.m_localPoint.y = T * f.col2.x + e * f.col2.y),
                g.m_id.Set(r.id),
                (g.m_id.features.flip = d),
                ++B);
          b.m_pointCount = B;
        }
      }
    }
  };
  H.CollideCircles = function (b, a, k, f, q) {
    b.m_pointCount = 0;
    var m = k.R;
    var p = a.m_p;
    var B = k.position.x + (m.col1.x * p.x + m.col2.x * p.y);
    k = k.position.y + (m.col1.y * p.x + m.col2.y * p.y);
    m = q.R;
    p = f.m_p;
    B = q.position.x + (m.col1.x * p.x + m.col2.x * p.y) - B;
    q = q.position.y + (m.col1.y * p.x + m.col2.y * p.y) - k;
    m = a.m_radius + f.m_radius;
    B * B + q * q > m * m ||
      ((b.m_type = D.e_circles),
      b.m_localPoint.SetV(a.m_p),
      b.m_localPlaneNormal.SetZero(),
      (b.m_pointCount = 1),
      b.m_points[0].m_localPoint.SetV(f.m_p),
      (b.m_points[0].m_id.key = 0));
  };
  H.CollidePolygonAndCircle = function (b, a, k, f, q) {
    b.m_pointCount = 0;
    var m = q.R;
    var p = f.m_p;
    var d = q.position.y + (m.col1.y * p.x + m.col2.y * p.y);
    var B = q.position.x + (m.col1.x * p.x + m.col2.x * p.y) - k.position.x;
    var r = d - k.position.y;
    m = k.R;
    k = B * m.col1.x + r * m.col1.y;
    m = B * m.col2.x + r * m.col2.y;
    var z = 0;
    q = -Number.MAX_VALUE;
    var d = a.m_radius + f.m_radius,
      v = parseInt(a.m_vertexCount),
      g = a.m_vertices;
    a = a.m_normals;
    for (var e = 0; e < v; ++e) {
      p = g[e];
      B = k - p.x;
      r = m - p.y;
      p = a[e];
      p = p.x * B + p.y * r;
      if (p > d) return;
      p > q && ((q = p), (z = e));
    }
    p = parseInt(z);
    r = parseInt(p + 1 < v ? p + 1 : 0);
    B = g[p];
    g = g[r];
    if (q < Number.MIN_VALUE)
      (b.m_pointCount = 1),
        (b.m_type = D.e_faceA),
        b.m_localPlaneNormal.SetV(a[z]),
        (b.m_localPoint.x = 0.5 * (B.x + g.x)),
        (b.m_localPoint.y = 0.5 * (B.y + g.y));
    else if (
      ((q = (k - g.x) * (B.x - g.x) + (m - g.y) * (B.y - g.y)),
      0 >= (k - B.x) * (g.x - B.x) + (m - B.y) * (g.y - B.y))
    ) {
      if ((k - B.x) * (k - B.x) + (m - B.y) * (m - B.y) > d * d) return;
      b.m_pointCount = 1;
      b.m_type = D.e_faceA;
      b.m_localPlaneNormal.x = k - B.x;
      b.m_localPlaneNormal.y = m - B.y;
      b.m_localPlaneNormal.Normalize();
      b.m_localPoint.SetV(B);
    } else if (0 >= q) {
      if ((k - g.x) * (k - g.x) + (m - g.y) * (m - g.y) > d * d) return;
      b.m_pointCount = 1;
      b.m_type = D.e_faceA;
      b.m_localPlaneNormal.x = k - g.x;
      b.m_localPlaneNormal.y = m - g.y;
      b.m_localPlaneNormal.Normalize();
      b.m_localPoint.SetV(g);
    } else {
      z = 0.5 * (B.x + g.x);
      g = 0.5 * (B.y + g.y);
      q = (k - z) * a[p].x + (m - g) * a[p].y;
      if (q > d) return;
      b.m_pointCount = 1;
      b.m_type = D.e_faceA;
      b.m_localPlaneNormal.x = a[p].x;
      b.m_localPlaneNormal.y = a[p].y;
      b.m_localPlaneNormal.Normalize();
      b.m_localPoint.Set(z, g);
    }
    b.m_points[0].m_localPoint.SetV(f.m_p);
    b.m_points[0].m_id.key = 0;
  };
  H.TestOverlap = function (b, a) {
    var m = a.lowerBound,
      k = b.upperBound,
      p = m.x - k.x,
      q = m.y - k.y,
      m = b.lowerBound,
      k = a.upperBound,
      f = m.y - k.y;
    return 0 < p || 0 < q || 0 < m.x - k.x || 0 < f ? !1 : !0;
  };
  Box2D.postDefs.push(function () {
    Box2D.Collision.b2Collision.s_incidentEdge = H.MakeClipPointVector();
    Box2D.Collision.b2Collision.s_clipPoints1 = H.MakeClipPointVector();
    Box2D.Collision.b2Collision.s_clipPoints2 = H.MakeClipPointVector();
    Box2D.Collision.b2Collision.s_edgeAO = new Vector_a2j_Number(1);
    Box2D.Collision.b2Collision.s_edgeBO = new Vector_a2j_Number(1);
    Box2D.Collision.b2Collision.s_localTangent = new e();
    Box2D.Collision.b2Collision.s_localNormal = new e();
    Box2D.Collision.b2Collision.s_planePoint = new e();
    Box2D.Collision.b2Collision.s_normal = new e();
    Box2D.Collision.b2Collision.s_tangent = new e();
    Box2D.Collision.b2Collision.s_tangent2 = new e();
    Box2D.Collision.b2Collision.s_v11 = new e();
    Box2D.Collision.b2Collision.s_v12 = new e();
    Box2D.Collision.b2Collision.b2CollidePolyTempVec = new e();
    Box2D.Collision.b2Collision.b2_nullFeature = 255;
  });
  L.b2ContactID = function () {
    this.features = new k();
  };
  L.prototype.b2ContactID = function () {
    this.features._m_id = this;
  };
  L.prototype.Set = function (b) {
    this.key = b._key;
  };
  L.prototype.Copy = function () {
    var b = new L();
    b.key = this.key;
    return b;
  };
  Object.defineProperty(L.prototype, "key", {
    enumerable: !1,
    configurable: !0,
    get: function () {
      return this._key;
    },
  });
  Object.defineProperty(L.prototype, "key", {
    enumerable: !1,
    configurable: !0,
    set: function (b) {
      void 0 === b && (b = 0);
      this._key = b;
      this.features._referenceEdge = this._key & 255;
      this.features._incidentEdge = ((this._key & 65280) >> 8) & 255;
      this.features._incidentVertex = ((this._key & 16711680) >> 16) & 255;
      this.features._flip = ((this._key & 4278190080) >> 24) & 255;
    },
  });
  y.b2ContactPoint = function () {
    this.position = new e();
    this.velocity = new e();
    this.normal = new e();
    this.id = new L();
  };
  Q.b2Distance = function () {};
  Q.Distance = function (b, a, k) {
    ++Q.b2_gjkCalls;
    var m = k.proxyA,
      p = k.proxyB,
      q = k.transformA,
      f = k.transformB,
      d = Q.s_simplex;
    d.ReadCache(a, m, q, p, f);
    var B = d.m_vertices,
      r = Q.s_saveA,
      z = Q.s_saveB;
    d.GetClosestPoint().LengthSquared();
    for (var g, v, J = 0; 20 > J; ) {
      var u = d.m_count;
      for (g = 0; g < u; g++) (r[g] = B[g].indexA), (z[g] = B[g].indexB);
      switch (d.m_count) {
        case 1:
          break;
        case 2:
          d.Solve2();
          break;
        case 3:
          d.Solve3();
          break;
        default:
          c.b2Assert(!1);
      }
      if (3 == d.m_count) break;
      v = d.GetClosestPoint();
      v.LengthSquared();
      g = d.GetSearchDirection();
      if (g.LengthSquared() < Number.MIN_VALUE * Number.MIN_VALUE) break;
      v = B[d.m_count];
      v.indexA = m.GetSupport(h.MulTMV(q.R, g.GetNegative()));
      v.wA = h.MulX(q, m.GetVertex(v.indexA));
      v.indexB = p.GetSupport(h.MulTMV(f.R, g));
      v.wB = h.MulX(f, p.GetVertex(v.indexB));
      v.w = h.SubtractVV(v.wB, v.wA);
      ++J;
      ++Q.b2_gjkIters;
      var l = !1;
      for (g = 0; g < u; g++)
        if (v.indexA == r[g] && v.indexB == z[g]) {
          l = !0;
          break;
        }
      if (l) break;
      ++d.m_count;
    }
    Q.b2_gjkMaxIters = h.Max(Q.b2_gjkMaxIters, J);
    d.GetWitnessPoints(b.pointA, b.pointB);
    b.distance = h.SubtractVV(b.pointA, b.pointB).Length();
    b.iterations = J;
    d.WriteCache(a);
    k.useRadii &&
      ((a = m.m_radius),
      (p = p.m_radius),
      b.distance > a + p && b.distance > Number.MIN_VALUE
        ? ((b.distance -= a + p),
          (k = h.SubtractVV(b.pointB, b.pointA)),
          k.Normalize(),
          (b.pointA.x += a * k.x),
          (b.pointA.y += a * k.y),
          (b.pointB.x -= p * k.x),
          (b.pointB.y -= p * k.y))
        : ((v = new e()),
          (v.x = 0.5 * (b.pointA.x + b.pointB.x)),
          (v.y = 0.5 * (b.pointA.y + b.pointB.y)),
          (b.pointA.x = b.pointB.x = v.x),
          (b.pointA.y = b.pointB.y = v.y),
          (b.distance = 0)));
  };
  Box2D.postDefs.push(function () {
    Box2D.Collision.b2Distance.s_simplex = new d();
    Box2D.Collision.b2Distance.s_saveA = new Vector_a2j_Number(3);
    Box2D.Collision.b2Distance.s_saveB = new Vector_a2j_Number(3);
  });
  u.b2DistanceInput = function () {};
  g.b2DistanceOutput = function () {
    this.pointA = new e();
    this.pointB = new e();
  };
  C.b2DistanceProxy = function () {};
  C.prototype.Set = function (b) {
    switch (b.GetType()) {
      case t.e_circleShape:
        b = b instanceof n ? b : null;
        this.m_vertices = new Vector(1, !0);
        this.m_vertices[0] = b.m_p;
        this.m_count = 1;
        this.m_radius = b.m_radius;
        break;
      case t.e_polygonShape:
        b = b instanceof x ? b : null;
        this.m_vertices = b.m_vertices;
        this.m_count = b.m_vertexCount;
        this.m_radius = b.m_radius;
        break;
      default:
        c.b2Assert(!1);
    }
  };
  C.prototype.GetSupport = function (b) {
    for (
      var k = 0,
        a = this.m_vertices[0].x * b.x + this.m_vertices[0].y * b.y,
        m = 1;
      m < this.m_count;
      ++m
    ) {
      var q = this.m_vertices[m].x * b.x + this.m_vertices[m].y * b.y;
      q > a && ((k = m), (a = q));
    }
    return k;
  };
  C.prototype.GetSupportVertex = function (b) {
    for (
      var k = 0,
        a = this.m_vertices[0].x * b.x + this.m_vertices[0].y * b.y,
        m = 1;
      m < this.m_count;
      ++m
    ) {
      var q = this.m_vertices[m].x * b.x + this.m_vertices[m].y * b.y;
      q > a && ((k = m), (a = q));
    }
    return this.m_vertices[k];
  };
  C.prototype.GetVertexCount = function () {
    return this.m_count;
  };
  C.prototype.GetVertex = function (b) {
    void 0 === b && (b = 0);
    c.b2Assert(0 <= b && b < this.m_count);
    return this.m_vertices[b];
  };
  E.b2DynamicTree = function () {};
  E.prototype.b2DynamicTree = function () {
    this.m_freeList = this.m_root = null;
    this.m_insertionCount = this.m_path = 0;
  };
  E.prototype.CreateProxy = function (b, k) {
    var a = this.AllocateNode(),
      m = c.b2_aabbExtension,
      p = c.b2_aabbExtension;
    a.aabb.lowerBound.x = b.lowerBound.x - m;
    a.aabb.lowerBound.y = b.lowerBound.y - p;
    a.aabb.upperBound.x = b.upperBound.x + m;
    a.aabb.upperBound.y = b.upperBound.y + p;
    a.userData = k;
    this.InsertLeaf(a);
    return a;
  };
  E.prototype.DestroyProxy = function (b) {
    this.RemoveLeaf(b);
    this.FreeNode(b);
  };
  E.prototype.MoveProxy = function (b, a, k) {
    c.b2Assert(b.IsLeaf());
    if (b.aabb.Contains(a)) return !1;
    this.RemoveLeaf(b);
    var m = c.b2_aabbExtension + c.b2_aabbMultiplier * (0 < k.x ? k.x : -k.x);
    k = c.b2_aabbExtension + c.b2_aabbMultiplier * (0 < k.y ? k.y : -k.y);
    b.aabb.lowerBound.x = a.lowerBound.x - m;
    b.aabb.lowerBound.y = a.lowerBound.y - k;
    b.aabb.upperBound.x = a.upperBound.x + m;
    b.aabb.upperBound.y = a.upperBound.y + k;
    this.InsertLeaf(b);
    return !0;
  };
  E.prototype.Rebalance = function (b) {
    void 0 === b && (b = 0);
    if (null != this.m_root)
      for (var k = 0; k < b; k++) {
        for (var a = this.m_root, m = 0; 0 == a.IsLeaf(); )
          (a = (this.m_path >> m) & 1 ? a.child2 : a.child1),
            (m = (m + 1) & 31);
        ++this.m_path;
        this.RemoveLeaf(a);
        this.InsertLeaf(a);
      }
  };
  E.prototype.GetFatAABB = function (b) {
    return b.aabb;
  };
  E.prototype.GetUserData = function (b) {
    return b.userData;
  };
  E.prototype.Query = function (b, a) {
    if (null != this.m_root) {
      var k = new Vector(),
        m = 0;
      for (k[m++] = this.m_root; 0 < m; ) {
        var p = k[--m];
        if (p.aabb.TestOverlap(a))
          if (p.IsLeaf()) {
            if (!b(p)) break;
          } else (k[m++] = p.child1), (k[m++] = p.child2);
      }
    }
  };
  E.prototype.RayCast = function (b, k) {
    if (null != this.m_root) {
      var a = k.p1,
        m = k.p2,
        p = h.SubtractVV(a, m);
      p.Normalize();
      var p = h.CrossFV(1, p),
        q = h.AbsV(p),
        f = k.maxFraction,
        d = new w();
      var r = a.x + f * (m.x - a.x);
      var v = a.y + f * (m.y - a.y);
      d.lowerBound.x = Math.min(a.x, r);
      d.lowerBound.y = Math.min(a.y, v);
      d.upperBound.x = Math.max(a.x, r);
      d.upperBound.y = Math.max(a.y, v);
      var g = new Vector(),
        z = 0;
      for (g[z++] = this.m_root; 0 < z; )
        if (((r = g[--z]), 0 != r.aabb.TestOverlap(d))) {
          v = r.aabb.GetCenter();
          var e = r.aabb.GetExtents();
          if (
            !(
              0 <
              Math.abs(p.x * (a.x - v.x) + p.y * (a.y - v.y)) -
                q.x * e.x -
                q.y * e.y
            )
          )
            if (r.IsLeaf()) {
              v = new I();
              v.p1 = k.p1;
              v.p2 = k.p2;
              v.maxFraction = f;
              r = b(v, r);
              if (0 == r) break;
              0 < r &&
                ((f = r),
                (r = a.x + f * (m.x - a.x)),
                (v = a.y + f * (m.y - a.y)),
                (d.lowerBound.x = Math.min(a.x, r)),
                (d.lowerBound.y = Math.min(a.y, v)),
                (d.upperBound.x = Math.max(a.x, r)),
                (d.upperBound.y = Math.max(a.y, v)));
            } else (g[z++] = r.child1), (g[z++] = r.child2);
        }
    }
  };
  E.prototype.AllocateNode = function () {
    if (this.m_freeList) {
      var b = this.m_freeList;
      this.m_freeList = b.parent;
      b.parent = null;
      b.child1 = null;
      b.child2 = null;
      return b;
    }
    return new K();
  };
  E.prototype.FreeNode = function (b) {
    b.parent = this.m_freeList;
    this.m_freeList = b;
  };
  E.prototype.InsertLeaf = function (b) {
    ++this.m_insertionCount;
    if (null == this.m_root) (this.m_root = b), (this.m_root.parent = null);
    else {
      var a = b.aabb.GetCenter(),
        k = this.m_root;
      if (0 == k.IsLeaf()) {
        do
          var m = k.child1,
            k = k.child2,
            k =
              Math.abs((m.aabb.lowerBound.x + m.aabb.upperBound.x) / 2 - a.x) +
                Math.abs(
                  (m.aabb.lowerBound.y + m.aabb.upperBound.y) / 2 - a.y
                ) <
              Math.abs((k.aabb.lowerBound.x + k.aabb.upperBound.x) / 2 - a.x) +
                Math.abs((k.aabb.lowerBound.y + k.aabb.upperBound.y) / 2 - a.y)
                ? m
                : k;
        while (0 == k.IsLeaf());
      }
      a = k.parent;
      m = this.AllocateNode();
      m.parent = a;
      m.userData = null;
      m.aabb.Combine(b.aabb, k.aabb);
      if (a) {
        k.parent.child1 == k ? (a.child1 = m) : (a.child2 = m);
        m.child1 = k;
        m.child2 = b;
        k.parent = m;
        b.parent = m;
        do {
          if (a.aabb.Contains(m.aabb)) break;
          a.aabb.Combine(a.child1.aabb, a.child2.aabb);
          m = a;
          a = a.parent;
        } while (a);
      } else
        (m.child1 = k),
          (m.child2 = b),
          (k.parent = m),
          (this.m_root = b.parent = m);
    }
  };
  E.prototype.RemoveLeaf = function (b) {
    if (b == this.m_root) this.m_root = null;
    else {
      var a = b.parent,
        k = a.parent;
      b = a.child1 == b ? a.child2 : a.child1;
      if (k)
        for (
          k.child1 == a ? (k.child1 = b) : (k.child2 = b),
            b.parent = k,
            this.FreeNode(a);
          k;

        ) {
          a = k.aabb;
          k.aabb = w.Combine(k.child1.aabb, k.child2.aabb);
          if (a.Contains(k.aabb)) break;
          k = k.parent;
        }
      else (this.m_root = b), (b.parent = null), this.FreeNode(a);
    }
  };
  M.b2DynamicTreeBroadPhase = function () {
    this.m_tree = new E();
    this.m_moveBuffer = new Vector();
    this.m_pairBuffer = new Vector();
    this.m_pairCount = 0;
  };
  M.prototype.CreateProxy = function (b, a) {
    var k = this.m_tree.CreateProxy(b, a);
    ++this.m_proxyCount;
    this.BufferMove(k);
    return k;
  };
  M.prototype.DestroyProxy = function (b) {
    this.UnBufferMove(b);
    --this.m_proxyCount;
    this.m_tree.DestroyProxy(b);
  };
  M.prototype.MoveProxy = function (b, a, k) {
    this.m_tree.MoveProxy(b, a, k) && this.BufferMove(b);
  };
  M.prototype.TestOverlap = function (b, a) {
    var k = this.m_tree.GetFatAABB(b),
      m = this.m_tree.GetFatAABB(a);
    return k.TestOverlap(m);
  };
  M.prototype.GetUserData = function (b) {
    return this.m_tree.GetUserData(b);
  };
  M.prototype.GetFatAABB = function (b) {
    return this.m_tree.GetFatAABB(b);
  };
  M.prototype.GetProxyCount = function () {
    return this.m_proxyCount;
  };
  M.prototype.UpdatePairs = function (b) {
    for (
      var a = this, k = (a.m_pairCount = 0), m, k = 0;
      k < a.m_moveBuffer.length;
      ++k
    ) {
      m = a.m_moveBuffer[k];
      var q = a.m_tree.GetFatAABB(m);
      a.m_tree.Query(function (b) {
        if (b == m) return !0;
        a.m_pairCount == a.m_pairBuffer.length &&
          (a.m_pairBuffer[a.m_pairCount] = new N());
        var k = a.m_pairBuffer[a.m_pairCount];
        k.proxyA = b < m ? b : m;
        k.proxyB = b >= m ? b : m;
        ++a.m_pairCount;
        return !0;
      }, q);
    }
    for (k = a.m_moveBuffer.length = 0; k < a.m_pairCount; ) {
      var q = a.m_pairBuffer[k],
        f = a.m_tree.GetUserData(q.proxyA),
        d = a.m_tree.GetUserData(q.proxyB);
      b(f, d);
      for (++k; k < a.m_pairCount; ) {
        f = a.m_pairBuffer[k];
        if (f.proxyA != q.proxyA || f.proxyB != q.proxyB) break;
        ++k;
      }
    }
  };
  M.prototype.Query = function (b, a) {
    this.m_tree.Query(b, a);
  };
  M.prototype.RayCast = function (b, a) {
    this.m_tree.RayCast(b, a);
  };
  M.prototype.Validate = function () {};
  M.prototype.Rebalance = function (b) {
    void 0 === b && (b = 0);
    this.m_tree.Rebalance(b);
  };
  M.prototype.BufferMove = function (b) {
    this.m_moveBuffer[this.m_moveBuffer.length] = b;
  };
  M.prototype.UnBufferMove = function (b) {
    b = parseInt(this.m_moveBuffer.indexOf(b));
    this.m_moveBuffer.splice(b, 1);
  };
  M.prototype.ComparePairs = function (b, a) {
    return 0;
  };
  M.__implements = {};
  M.__implements[q] = !0;
  K.b2DynamicTreeNode = function () {
    this.aabb = new w();
  };
  K.prototype.IsLeaf = function () {
    return null == this.child1;
  };
  N.b2DynamicTreePair = function () {};
  D.b2Manifold = function () {
    this.m_pointCount = 0;
  };
  D.prototype.b2Manifold = function () {
    this.m_points = new Vector(c.b2_maxManifoldPoints);
    for (var b = 0; b < c.b2_maxManifoldPoints; b++) this.m_points[b] = new R();
    this.m_localPlaneNormal = new e();
    this.m_localPoint = new e();
  };
  D.prototype.Reset = function () {
    for (var b = 0; b < c.b2_maxManifoldPoints; b++)
      (this.m_points[b] instanceof R ? this.m_points[b] : null).Reset();
    this.m_localPlaneNormal.SetZero();
    this.m_localPoint.SetZero();
    this.m_pointCount = this.m_type = 0;
  };
  D.prototype.Set = function (b) {
    this.m_pointCount = b.m_pointCount;
    for (var a = 0; a < c.b2_maxManifoldPoints; a++)
      (this.m_points[a] instanceof R ? this.m_points[a] : null).Set(
        b.m_points[a]
      );
    this.m_localPlaneNormal.SetV(b.m_localPlaneNormal);
    this.m_localPoint.SetV(b.m_localPoint);
    this.m_type = b.m_type;
  };
  D.prototype.Copy = function () {
    var b = new D();
    b.Set(this);
    return b;
  };
  Box2D.postDefs.push(function () {
    Box2D.Collision.b2Manifold.e_circles = 1;
    Box2D.Collision.b2Manifold.e_faceA = 2;
    Box2D.Collision.b2Manifold.e_faceB = 4;
  });
  R.b2ManifoldPoint = function () {
    this.m_localPoint = new e();
    this.m_id = new L();
  };
  R.prototype.b2ManifoldPoint = function () {
    this.Reset();
  };
  R.prototype.Reset = function () {
    this.m_localPoint.SetZero();
    this.m_tangentImpulse = this.m_normalImpulse = 0;
    this.m_id.key = 0;
  };
  R.prototype.Set = function (b) {
    this.m_localPoint.SetV(b.m_localPoint);
    this.m_normalImpulse = b.m_normalImpulse;
    this.m_tangentImpulse = b.m_tangentImpulse;
    this.m_id.Set(b.m_id);
  };
  F.b2Point = function () {
    this.p = new e();
  };
  F.prototype.Support = function (b, a, k) {
    return this.p;
  };
  F.prototype.GetFirstVertex = function (b) {
    return this.p;
  };
  I.b2RayCastInput = function () {
    this.p1 = new e();
    this.p2 = new e();
  };
  I.prototype.b2RayCastInput = function (b, a, k) {
    void 0 === b && (b = null);
    void 0 === a && (a = null);
    void 0 === k && (k = 1);
    b && this.p1.SetV(b);
    a && this.p2.SetV(a);
    this.maxFraction = k;
  };
  S.b2RayCastOutput = function () {
    this.normal = new e();
  };
  O.b2Segment = function () {
    this.p1 = new e();
    this.p2 = new e();
  };
  O.prototype.TestSegment = function (b, a, k, q) {
    void 0 === q && (q = 0);
    var m = k.p1,
      f = k.p2.x - m.x,
      p = k.p2.y - m.y;
    k = this.p2.y - this.p1.y;
    var d = -(this.p2.x - this.p1.x),
      r = 100 * Number.MIN_VALUE,
      v = -(f * k + p * d);
    if (v > r) {
      var g = m.x - this.p1.x,
        z = m.y - this.p1.y,
        m = g * k + z * d;
      if (
        0 <= m &&
        m <= q * v &&
        ((q = -f * z + p * g), -r * v <= q && q <= v * (1 + r))
      )
        return (
          (q = Math.sqrt(k * k + d * d)),
          (b[0] = m / v),
          a.Set(k / q, d / q),
          !0
        );
    }
    return !1;
  };
  O.prototype.Extend = function (b) {
    this.ExtendForward(b);
    this.ExtendBackward(b);
  };
  O.prototype.ExtendForward = function (b) {
    var a = this.p2.x - this.p1.x,
      k = this.p2.y - this.p1.y;
    b = Math.min(
      0 < a
        ? (b.upperBound.x - this.p1.x) / a
        : 0 > a
        ? (b.lowerBound.x - this.p1.x) / a
        : Number.POSITIVE_INFINITY,
      0 < k
        ? (b.upperBound.y - this.p1.y) / k
        : 0 > k
        ? (b.lowerBound.y - this.p1.y) / k
        : Number.POSITIVE_INFINITY
    );
    this.p2.x = this.p1.x + a * b;
    this.p2.y = this.p1.y + k * b;
  };
  O.prototype.ExtendBackward = function (b) {
    var a = -this.p2.x + this.p1.x,
      k = -this.p2.y + this.p1.y;
    b = Math.min(
      0 < a
        ? (b.upperBound.x - this.p2.x) / a
        : 0 > a
        ? (b.lowerBound.x - this.p2.x) / a
        : Number.POSITIVE_INFINITY,
      0 < k
        ? (b.upperBound.y - this.p2.y) / k
        : 0 > k
        ? (b.lowerBound.y - this.p2.y) / k
        : Number.POSITIVE_INFINITY
    );
    this.p1.x = this.p2.x + a * b;
    this.p1.y = this.p2.y + k * b;
  };
  a.b2SeparationFunction = function () {
    this.m_localPoint = new e();
    this.m_axis = new e();
  };
  a.prototype.Initialize = function (b, k, q, f, d) {
    this.m_proxyA = k;
    this.m_proxyB = f;
    k = parseInt(b.count);
    c.b2Assert(0 < k && 3 > k);
    if (1 == k) {
      this.m_type = a.e_points;
      var m = this.m_proxyA.GetVertex(b.indexA[0]);
      var p = this.m_proxyB.GetVertex(b.indexB[0]);
      var r = m;
      var v = q.R;
      m = q.position.x + (v.col1.x * r.x + v.col2.x * r.y);
      q = q.position.y + (v.col1.y * r.x + v.col2.y * r.y);
      r = p;
      v = d.R;
      p = d.position.x + (v.col1.x * r.x + v.col2.x * r.y);
      d = d.position.y + (v.col1.y * r.x + v.col2.y * r.y);
      this.m_axis.x = p - m;
      this.m_axis.y = d - q;
      this.m_axis.Normalize();
    } else if (b.indexB[0] == b.indexB[1])
      (this.m_type = a.e_faceA),
        (k = this.m_proxyA.GetVertex(b.indexA[0])),
        (f = this.m_proxyA.GetVertex(b.indexA[1])),
        (p = this.m_proxyB.GetVertex(b.indexB[0])),
        (this.m_localPoint.x = 0.5 * (k.x + f.x)),
        (this.m_localPoint.y = 0.5 * (k.y + f.y)),
        (this.m_axis = h.CrossVF(h.SubtractVV(f, k), 1)),
        this.m_axis.Normalize(),
        (r = this.m_axis),
        (v = q.R),
        (k = v.col1.x * r.x + v.col2.x * r.y),
        (f = v.col1.y * r.x + v.col2.y * r.y),
        (r = this.m_localPoint),
        (v = q.R),
        (m = q.position.x + (v.col1.x * r.x + v.col2.x * r.y)),
        (q = q.position.y + (v.col1.y * r.x + v.col2.y * r.y)),
        (r = p),
        (v = d.R),
        (p = d.position.x + (v.col1.x * r.x + v.col2.x * r.y)),
        (d = d.position.y + (v.col1.y * r.x + v.col2.y * r.y)),
        0 > (p - m) * k + (d - q) * f && this.m_axis.NegativeSelf();
    else if (b.indexA[0] == b.indexA[0])
      (this.m_type = a.e_faceB),
        (v = this.m_proxyB.GetVertex(b.indexB[0])),
        (r = this.m_proxyB.GetVertex(b.indexB[1])),
        (m = this.m_proxyA.GetVertex(b.indexA[0])),
        (this.m_localPoint.x = 0.5 * (v.x + r.x)),
        (this.m_localPoint.y = 0.5 * (v.y + r.y)),
        (this.m_axis = h.CrossVF(h.SubtractVV(r, v), 1)),
        this.m_axis.Normalize(),
        (r = this.m_axis),
        (v = d.R),
        (k = v.col1.x * r.x + v.col2.x * r.y),
        (f = v.col1.y * r.x + v.col2.y * r.y),
        (r = this.m_localPoint),
        (v = d.R),
        (p = d.position.x + (v.col1.x * r.x + v.col2.x * r.y)),
        (d = d.position.y + (v.col1.y * r.x + v.col2.y * r.y)),
        (r = m),
        (v = q.R),
        (m = q.position.x + (v.col1.x * r.x + v.col2.x * r.y)),
        (q = q.position.y + (v.col1.y * r.x + v.col2.y * r.y)),
        0 > (m - p) * k + (q - d) * f && this.m_axis.NegativeSelf();
    else {
      k = this.m_proxyA.GetVertex(b.indexA[0]);
      f = this.m_proxyA.GetVertex(b.indexA[1]);
      v = this.m_proxyB.GetVertex(b.indexB[0]);
      r = this.m_proxyB.GetVertex(b.indexB[1]);
      h.MulX(q, m);
      m = h.MulMV(q.R, h.SubtractVV(f, k));
      h.MulX(d, p);
      b = h.MulMV(d.R, h.SubtractVV(r, v));
      d = m.x * m.x + m.y * m.y;
      p = b.x * b.x + b.y * b.y;
      var g = h.SubtractVV(b, m);
      q = m.x * g.x + m.y * g.y;
      g = b.x * g.x + b.y * g.y;
      m = m.x * b.x + m.y * b.y;
      var z = d * p - m * m;
      b = 0;
      0 != z && (b = h.Clamp((m * g - q * p) / z, 0, 1));
      0 > (m * b + g) / p && (b = h.Clamp((m - q) / d, 0, 1));
      m = new e();
      m.x = k.x + b * (f.x - k.x);
      m.y = k.y + b * (f.y - k.y);
      p = new e();
      p.x = v.x + b * (r.x - v.x);
      p.y = v.y + b * (r.y - v.y);
      0 == b || 1 == b
        ? ((this.m_type = a.e_faceB),
          (this.m_axis = h.CrossVF(h.SubtractVV(r, v), 1)),
          this.m_axis.Normalize(),
          (this.m_localPoint = p))
        : ((this.m_type = a.e_faceA),
          (this.m_axis = h.CrossVF(h.SubtractVV(f, k), 1)),
          (this.m_localPoint = m));
      0 > b && this.m_axis.NegativeSelf();
    }
  };
  a.prototype.Evaluate = function (b, k) {
    var m;
    switch (this.m_type) {
      case a.e_points:
        var q = h.MulTMV(b.R, this.m_axis);
        var f = h.MulTMV(k.R, this.m_axis.GetNegative());
        q = this.m_proxyA.GetSupportVertex(q);
        f = this.m_proxyB.GetSupportVertex(f);
        q = h.MulX(b, q);
        f = h.MulX(k, f);
        return (m = (f.x - q.x) * this.m_axis.x + (f.y - q.y) * this.m_axis.y);
      case a.e_faceA:
        return (
          (m = h.MulMV(b.R, this.m_axis)),
          (q = h.MulX(b, this.m_localPoint)),
          (f = h.MulTMV(k.R, m.GetNegative())),
          (f = this.m_proxyB.GetSupportVertex(f)),
          (f = h.MulX(k, f)),
          (m = (f.x - q.x) * m.x + (f.y - q.y) * m.y)
        );
      case a.e_faceB:
        return (
          (m = h.MulMV(k.R, this.m_axis)),
          (f = h.MulX(k, this.m_localPoint)),
          (q = h.MulTMV(b.R, m.GetNegative())),
          (q = this.m_proxyA.GetSupportVertex(q)),
          (q = h.MulX(b, q)),
          (m = (q.x - f.x) * m.x + (q.y - f.y) * m.y)
        );
      default:
        return c.b2Assert(!1), 0;
    }
  };
  Box2D.postDefs.push(function () {
    Box2D.Collision.b2SeparationFunction.e_points = 1;
    Box2D.Collision.b2SeparationFunction.e_faceA = 2;
    Box2D.Collision.b2SeparationFunction.e_faceB = 4;
  });
  d.b2Simplex = function () {
    this.m_v1 = new f();
    this.m_v2 = new f();
    this.m_v3 = new f();
    this.m_vertices = new Vector(3);
  };
  d.prototype.b2Simplex = function () {
    this.m_vertices[0] = this.m_v1;
    this.m_vertices[1] = this.m_v2;
    this.m_vertices[2] = this.m_v3;
  };
  d.prototype.ReadCache = function (b, k, a, q, f) {
    c.b2Assert(0 <= b.count && 3 >= b.count);
    this.m_count = b.count;
    for (var m = this.m_vertices, d = 0; d < this.m_count; d++) {
      var p = m[d];
      p.indexA = b.indexA[d];
      p.indexB = b.indexB[d];
      var r = k.GetVertex(p.indexA);
      var v = q.GetVertex(p.indexB);
      p.wA = h.MulX(a, r);
      p.wB = h.MulX(f, v);
      p.w = h.SubtractVV(p.wB, p.wA);
      p.a = 0;
    }
    1 < this.m_count &&
      ((b = b.metric),
      (p = this.GetMetric()),
      p < 0.5 * b || 2 * b < p || p < Number.MIN_VALUE) &&
      (this.m_count = 0);
    0 == this.m_count &&
      ((p = m[0]),
      (p.indexA = 0),
      (p.indexB = 0),
      (r = k.GetVertex(0)),
      (v = q.GetVertex(0)),
      (p.wA = h.MulX(a, r)),
      (p.wB = h.MulX(f, v)),
      (p.w = h.SubtractVV(p.wB, p.wA)),
      (this.m_count = 1));
  };
  d.prototype.WriteCache = function (b) {
    b.metric = this.GetMetric();
    b.count = Box2D.parseUInt(this.m_count);
    for (var k = this.m_vertices, a = 0; a < this.m_count; a++)
      (b.indexA[a] = Box2D.parseUInt(k[a].indexA)),
        (b.indexB[a] = Box2D.parseUInt(k[a].indexB));
  };
  d.prototype.GetSearchDirection = function () {
    switch (this.m_count) {
      case 1:
        return this.m_v1.w.GetNegative();
      case 2:
        var b = h.SubtractVV(this.m_v2.w, this.m_v1.w);
        return 0 < h.CrossVV(b, this.m_v1.w.GetNegative())
          ? h.CrossFV(1, b)
          : h.CrossVF(b, 1);
      default:
        return c.b2Assert(!1), new e();
    }
  };
  d.prototype.GetClosestPoint = function () {
    switch (this.m_count) {
      case 0:
        return c.b2Assert(!1), new e();
      case 1:
        return this.m_v1.w;
      case 2:
        return new e(
          this.m_v1.a * this.m_v1.w.x + this.m_v2.a * this.m_v2.w.x,
          this.m_v1.a * this.m_v1.w.y + this.m_v2.a * this.m_v2.w.y
        );
      default:
        return c.b2Assert(!1), new e();
    }
  };
  d.prototype.GetWitnessPoints = function (b, k) {
    switch (this.m_count) {
      case 0:
        c.b2Assert(!1);
        break;
      case 1:
        b.SetV(this.m_v1.wA);
        k.SetV(this.m_v1.wB);
        break;
      case 2:
        b.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x;
        b.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y;
        k.x = this.m_v1.a * this.m_v1.wB.x + this.m_v2.a * this.m_v2.wB.x;
        k.y = this.m_v1.a * this.m_v1.wB.y + this.m_v2.a * this.m_v2.wB.y;
        break;
      case 3:
        k.x = b.x =
          this.m_v1.a * this.m_v1.wA.x +
          this.m_v2.a * this.m_v2.wA.x +
          this.m_v3.a * this.m_v3.wA.x;
        k.y = b.y =
          this.m_v1.a * this.m_v1.wA.y +
          this.m_v2.a * this.m_v2.wA.y +
          this.m_v3.a * this.m_v3.wA.y;
        break;
      default:
        c.b2Assert(!1);
    }
  };
  d.prototype.GetMetric = function () {
    switch (this.m_count) {
      case 0:
        return c.b2Assert(!1), 0;
      case 1:
        return 0;
      case 2:
        return h.SubtractVV(this.m_v1.w, this.m_v2.w).Length();
      case 3:
        return h.CrossVV(
          h.SubtractVV(this.m_v2.w, this.m_v1.w),
          h.SubtractVV(this.m_v3.w, this.m_v1.w)
        );
      default:
        return c.b2Assert(!1), 0;
    }
  };
  d.prototype.Solve2 = function () {
    var b = this.m_v1.w,
      k = this.m_v2.w,
      a = h.SubtractVV(k, b),
      b = -(b.x * a.x + b.y * a.y);
    0 >= b
      ? (this.m_count = this.m_v1.a = 1)
      : ((k = k.x * a.x + k.y * a.y),
        0 >= k
          ? ((this.m_count = this.m_v2.a = 1), this.m_v1.Set(this.m_v2))
          : ((a = 1 / (k + b)),
            (this.m_v1.a = k * a),
            (this.m_v2.a = b * a),
            (this.m_count = 2)));
  };
  d.prototype.Solve3 = function () {
    var b = this.m_v1.w,
      k = this.m_v2.w,
      a = this.m_v3.w,
      q = h.SubtractVV(k, b),
      f = h.Dot(b, q),
      d = h.Dot(k, q),
      f = -f,
      r = h.SubtractVV(a, b),
      v = h.Dot(b, r),
      g = h.Dot(a, r),
      v = -v,
      z = h.SubtractVV(a, k),
      e = h.Dot(k, z),
      z = h.Dot(a, z),
      e = -e,
      r = h.CrossVV(q, r),
      q = r * h.CrossVV(k, a),
      a = r * h.CrossVV(a, b),
      b = r * h.CrossVV(b, k);
    0 >= f && 0 >= v
      ? (this.m_count = this.m_v1.a = 1)
      : 0 < d && 0 < f && 0 >= b
      ? ((g = 1 / (d + f)),
        (this.m_v1.a = d * g),
        (this.m_v2.a = f * g),
        (this.m_count = 2))
      : 0 < g && 0 < v && 0 >= a
      ? ((d = 1 / (g + v)),
        (this.m_v1.a = g * d),
        (this.m_v3.a = v * d),
        (this.m_count = 2),
        this.m_v2.Set(this.m_v3))
      : 0 >= d && 0 >= e
      ? ((this.m_count = this.m_v2.a = 1), this.m_v1.Set(this.m_v2))
      : 0 >= g && 0 >= z
      ? ((this.m_count = this.m_v3.a = 1), this.m_v1.Set(this.m_v3))
      : 0 < z && 0 < e && 0 >= q
      ? ((d = 1 / (z + e)),
        (this.m_v2.a = z * d),
        (this.m_v3.a = e * d),
        (this.m_count = 2),
        this.m_v1.Set(this.m_v3))
      : ((d = 1 / (q + a + b)),
        (this.m_v1.a = q * d),
        (this.m_v2.a = a * d),
        (this.m_v3.a = b * d),
        (this.m_count = 3));
  };
  z.b2SimplexCache = function () {
    this.indexA = new Vector_a2j_Number(3);
    this.indexB = new Vector_a2j_Number(3);
  };
  f.b2SimplexVertex = function () {};
  f.prototype.Set = function (b) {
    this.wA.SetV(b.wA);
    this.wB.SetV(b.wB);
    this.w.SetV(b.w);
    this.a = b.a;
    this.indexA = b.indexA;
    this.indexB = b.indexB;
  };
  r.b2TimeOfImpact = function () {};
  r.TimeOfImpact = function (b) {
    ++r.b2_toiCalls;
    var k = b.proxyA,
      a = b.proxyB,
      q = b.sweepA,
      f = b.sweepB;
    c.b2Assert(q.t0 == f.t0);
    c.b2Assert(1 - q.t0 > Number.MIN_VALUE);
    var m = k.m_radius + a.m_radius;
    b = b.tolerance;
    var d = 0,
      v = 0,
      g = 0;
    r.s_cache.count = 0;
    for (r.s_distanceInput.useRadii = !1; ; ) {
      q.GetTransform(r.s_xfA, d);
      f.GetTransform(r.s_xfB, d);
      r.s_distanceInput.proxyA = k;
      r.s_distanceInput.proxyB = a;
      r.s_distanceInput.transformA = r.s_xfA;
      r.s_distanceInput.transformB = r.s_xfB;
      Q.Distance(r.s_distanceOutput, r.s_cache, r.s_distanceInput);
      if (0 >= r.s_distanceOutput.distance) {
        d = 1;
        break;
      }
      r.s_fcn.Initialize(r.s_cache, k, r.s_xfA, a, r.s_xfB);
      var z = r.s_fcn.Evaluate(r.s_xfA, r.s_xfB);
      if (0 >= z) {
        d = 1;
        break;
      }
      0 == v && (g = z > m ? h.Max(m - b, 0.75 * m) : h.Max(z - b, 0.02 * m));
      if (z - g < 0.5 * b) {
        if (0 == v) {
          d = 1;
          break;
        }
        break;
      }
      var e = d,
        J = d,
        u = 1;
      q.GetTransform(r.s_xfA, u);
      f.GetTransform(r.s_xfB, u);
      var l = r.s_fcn.Evaluate(r.s_xfA, r.s_xfB);
      if (l >= g) {
        d = 1;
        break;
      }
      for (var C = 0; ; ) {
        var w = C & 1 ? J + ((g - z) * (u - J)) / (l - z) : 0.5 * (J + u);
        q.GetTransform(r.s_xfA, w);
        f.GetTransform(r.s_xfB, w);
        var E = r.s_fcn.Evaluate(r.s_xfA, r.s_xfB);
        if (h.Abs(E - g) < 0.025 * b) {
          e = w;
          break;
        }
        E > g ? ((J = w), (z = E)) : ((u = w), (l = E));
        ++C;
        ++r.b2_toiRootIters;
        if (50 == C) break;
      }
      r.b2_toiMaxRootIters = h.Max(r.b2_toiMaxRootIters, C);
      if (e < (1 + 100 * Number.MIN_VALUE) * d) break;
      d = e;
      v++;
      ++r.b2_toiIters;
      if (1e3 == v) break;
    }
    r.b2_toiMaxIters = h.Max(r.b2_toiMaxIters, v);
    return d;
  };
  Box2D.postDefs.push(function () {
    Box2D.Collision.b2TimeOfImpact.b2_toiCalls = 0;
    Box2D.Collision.b2TimeOfImpact.b2_toiIters = 0;
    Box2D.Collision.b2TimeOfImpact.b2_toiMaxIters = 0;
    Box2D.Collision.b2TimeOfImpact.b2_toiRootIters = 0;
    Box2D.Collision.b2TimeOfImpact.b2_toiMaxRootIters = 0;
    Box2D.Collision.b2TimeOfImpact.s_cache = new z();
    Box2D.Collision.b2TimeOfImpact.s_distanceInput = new u();
    Box2D.Collision.b2TimeOfImpact.s_xfA = new A();
    Box2D.Collision.b2TimeOfImpact.s_xfB = new A();
    Box2D.Collision.b2TimeOfImpact.s_fcn = new a();
    Box2D.Collision.b2TimeOfImpact.s_distanceOutput = new g();
  });
  v.b2TOIInput = function () {
    this.proxyA = new C();
    this.proxyB = new C();
    this.sweepA = new l();
    this.sweepB = new l();
  };
  J.b2WorldManifold = function () {
    this.m_normal = new e();
  };
  J.prototype.b2WorldManifold = function () {
    this.m_points = new Vector(c.b2_maxManifoldPoints);
    for (var b = 0; b < c.b2_maxManifoldPoints; b++) this.m_points[b] = new e();
  };
  J.prototype.Initialize = function (b, k, a, q, f) {
    void 0 === a && (a = 0);
    void 0 === f && (f = 0);
    if (0 != b.m_pointCount)
      switch (b.m_type) {
        case D.e_circles:
          var d = k.R;
          var m = b.m_localPoint;
          var r = k.position.x + d.col1.x * m.x + d.col2.x * m.y;
          k = k.position.y + d.col1.y * m.x + d.col2.y * m.y;
          d = q.R;
          m = b.m_points[0].m_localPoint;
          b = q.position.x + d.col1.x * m.x + d.col2.x * m.y;
          q = q.position.y + d.col1.y * m.x + d.col2.y * m.y;
          m = b - r;
          d = q - k;
          var p = m * m + d * d;
          p > Number.MIN_VALUE * Number.MIN_VALUE
            ? ((p = Math.sqrt(p)),
              (this.m_normal.x = m / p),
              (this.m_normal.y = d / p))
            : ((this.m_normal.x = 1), (this.m_normal.y = 0));
          m = k + a * this.m_normal.y;
          q -= f * this.m_normal.y;
          this.m_points[0].x =
            0.5 * (r + a * this.m_normal.x + (b - f * this.m_normal.x));
          this.m_points[0].y = 0.5 * (m + q);
          break;
        case D.e_faceA:
          d = k.R;
          m = b.m_localPlaneNormal;
          p = d.col1.x * m.x + d.col2.x * m.y;
          var v = d.col1.y * m.x + d.col2.y * m.y;
          d = k.R;
          m = b.m_localPoint;
          var g = k.position.x + d.col1.x * m.x + d.col2.x * m.y;
          var z = k.position.y + d.col1.y * m.x + d.col2.y * m.y;
          this.m_normal.x = p;
          this.m_normal.y = v;
          for (r = 0; r < b.m_pointCount; r++) {
            d = q.R;
            m = b.m_points[r].m_localPoint;
            var e = q.position.x + d.col1.x * m.x + d.col2.x * m.y;
            m = q.position.y + d.col1.y * m.x + d.col2.y * m.y;
            this.m_points[r].x =
              e + 0.5 * (a - (e - g) * p - (m - z) * v - f) * p;
            this.m_points[r].y =
              m + 0.5 * (a - (e - g) * p - (m - z) * v - f) * v;
          }
          break;
        case D.e_faceB:
          for (
            d = q.R,
              m = b.m_localPlaneNormal,
              p = d.col1.x * m.x + d.col2.x * m.y,
              v = d.col1.y * m.x + d.col2.y * m.y,
              d = q.R,
              m = b.m_localPoint,
              g = q.position.x + d.col1.x * m.x + d.col2.x * m.y,
              z = q.position.y + d.col1.y * m.x + d.col2.y * m.y,
              this.m_normal.x = -p,
              this.m_normal.y = -v,
              r = 0;
            r < b.m_pointCount;
            r++
          )
            (d = k.R),
              (m = b.m_points[r].m_localPoint),
              (e = k.position.x + d.col1.x * m.x + d.col2.x * m.y),
              (m = k.position.y + d.col1.y * m.x + d.col2.y * m.y),
              (this.m_points[r].x =
                e + 0.5 * (f - (e - g) * p - (m - z) * v - a) * p),
              (this.m_points[r].y =
                m + 0.5 * (f - (e - g) * p - (m - z) * v - a) * v);
      }
  };
  b.ClipVertex = function () {
    this.v = new e();
    this.id = new L();
  };
  b.prototype.Set = function (b) {
    this.v.SetV(b.v);
    this.id.Set(b.id);
  };
  k.Features = function () {};
  Object.defineProperty(k.prototype, "referenceEdge", {
    enumerable: !1,
    configurable: !0,
    get: function () {
      return this._referenceEdge;
    },
  });
  Object.defineProperty(k.prototype, "referenceEdge", {
    enumerable: !1,
    configurable: !0,
    set: function (b) {
      void 0 === b && (b = 0);
      this._referenceEdge = b;
      this._m_id._key =
        (this._m_id._key & 4294967040) | (this._referenceEdge & 255);
    },
  });
  Object.defineProperty(k.prototype, "incidentEdge", {
    enumerable: !1,
    configurable: !0,
    get: function () {
      return this._incidentEdge;
    },
  });
  Object.defineProperty(k.prototype, "incidentEdge", {
    enumerable: !1,
    configurable: !0,
    set: function (b) {
      void 0 === b && (b = 0);
      this._incidentEdge = b;
      this._m_id._key =
        (this._m_id._key & 4294902015) | ((this._incidentEdge << 8) & 65280);
    },
  });
  Object.defineProperty(k.prototype, "incidentVertex", {
    enumerable: !1,
    configurable: !0,
    get: function () {
      return this._incidentVertex;
    },
  });
  Object.defineProperty(k.prototype, "incidentVertex", {
    enumerable: !1,
    configurable: !0,
    set: function (b) {
      void 0 === b && (b = 0);
      this._incidentVertex = b;
      this._m_id._key =
        (this._m_id._key & 4278255615) |
        ((this._incidentVertex << 16) & 16711680);
    },
  });
  Object.defineProperty(k.prototype, "flip", {
    enumerable: !1,
    configurable: !0,
    get: function () {
      return this._flip;
    },
  });
  Object.defineProperty(k.prototype, "flip", {
    enumerable: !1,
    configurable: !0,
    set: function (b) {
      void 0 === b && (b = 0);
      this._flip = b;
      this._m_id._key =
        (this._m_id._key & 16777215) | ((this._flip << 24) & 4278190080);
    },
  });
})();
(function () {
  var n = Box2D.Common.b2Settings,
    x = Box2D.Collision.Shapes.b2CircleShape,
    t = Box2D.Collision.Shapes.b2EdgeChainDef,
    c = Box2D.Collision.Shapes.b2EdgeShape,
    h = Box2D.Collision.Shapes.b2MassData,
    l = Box2D.Collision.Shapes.b2PolygonShape,
    A = Box2D.Collision.Shapes.b2Shape,
    e = Box2D.Common.Math.b2Math,
    w = Box2D.Common.Math.b2Transform,
    G = Box2D.Common.Math.b2Vec2,
    P = Box2D.Collision.b2Distance,
    H = Box2D.Collision.b2DistanceInput,
    L = Box2D.Collision.b2DistanceOutput,
    y = Box2D.Collision.b2DistanceProxy,
    Q = Box2D.Collision.b2SimplexCache;
  Box2D.inherit(x, Box2D.Collision.Shapes.b2Shape);
  x.prototype.__super = Box2D.Collision.Shapes.b2Shape.prototype;
  x.b2CircleShape = function () {
    Box2D.Collision.Shapes.b2Shape.b2Shape.apply(this, arguments);
    this.m_p = new G();
  };
  x.prototype.Copy = function () {
    var e = new x();
    e.Set(this);
    return e;
  };
  x.prototype.Set = function (e) {
    this.__super.Set.call(this, e);
    Box2D.is(e, x) && this.m_p.SetV((e instanceof x ? e : null).m_p);
  };
  x.prototype.TestPoint = function (e, g) {
    var u = e.R,
      c = e.position.x + (u.col1.x * this.m_p.x + u.col2.x * this.m_p.y),
      u = e.position.y + (u.col1.y * this.m_p.x + u.col2.y * this.m_p.y),
      c = g.x - c,
      u = g.y - u;
    return c * c + u * u <= this.m_radius * this.m_radius;
  };
  x.prototype.RayCast = function (e, g, c) {
    var u = c.R,
      h =
        g.p1.x -
        (c.position.x + (u.col1.x * this.m_p.x + u.col2.x * this.m_p.y));
    c =
      g.p1.y - (c.position.y + (u.col1.y * this.m_p.x + u.col2.y * this.m_p.y));
    var u = g.p2.x - g.p1.x,
      l = g.p2.y - g.p1.y,
      w = h * u + c * l,
      C = u * u + l * l,
      n = w * w - C * (h * h + c * c - this.m_radius * this.m_radius);
    if (0 > n || C < Number.MIN_VALUE) return !1;
    w = -(w + Math.sqrt(n));
    return 0 <= w && w <= g.maxFraction * C
      ? ((w /= C),
        (e.fraction = w),
        (e.normal.x = h + w * u),
        (e.normal.y = c + w * l),
        e.normal.Normalize(),
        !0)
      : !1;
  };
  x.prototype.ComputeAABB = function (e, g) {
    var u = g.R,
      c = g.position.x + (u.col1.x * this.m_p.x + u.col2.x * this.m_p.y),
      u = g.position.y + (u.col1.y * this.m_p.x + u.col2.y * this.m_p.y);
    e.lowerBound.Set(c - this.m_radius, u - this.m_radius);
    e.upperBound.Set(c + this.m_radius, u + this.m_radius);
  };
  x.prototype.ComputeMass = function (e, g) {
    void 0 === g && (g = 0);
    e.mass = g * n.b2_pi * this.m_radius * this.m_radius;
    e.center.SetV(this.m_p);
    e.I =
      e.mass *
      (0.5 * this.m_radius * this.m_radius +
        (this.m_p.x * this.m_p.x + this.m_p.y * this.m_p.y));
  };
  x.prototype.ComputeSubmergedArea = function (u, g, c, h) {
    void 0 === g && (g = 0);
    c = e.MulX(c, this.m_p);
    var l = -(e.Dot(u, c) - g);
    if (l < -this.m_radius + Number.MIN_VALUE) return 0;
    if (l > this.m_radius)
      return h.SetV(c), Math.PI * this.m_radius * this.m_radius;
    g = this.m_radius * this.m_radius;
    var w = l * l,
      l =
        g * (Math.asin(l / this.m_radius) + Math.PI / 2) + l * Math.sqrt(g - w);
    g = ((-2 / 3) * Math.pow(g - w, 1.5)) / l;
    h.x = c.x + u.x * g;
    h.y = c.y + u.y * g;
    return l;
  };
  x.prototype.GetLocalPosition = function () {
    return this.m_p;
  };
  x.prototype.SetLocalPosition = function (e) {
    this.m_p.SetV(e);
  };
  x.prototype.GetRadius = function () {
    return this.m_radius;
  };
  x.prototype.SetRadius = function (e) {
    void 0 === e && (e = 0);
    this.m_radius = e;
  };
  x.prototype.b2CircleShape = function (e) {
    void 0 === e && (e = 0);
    this.__super.b2Shape.call(this);
    this.m_type = A.e_circleShape;
    this.m_radius = e;
  };
  t.b2EdgeChainDef = function () {};
  t.prototype.b2EdgeChainDef = function () {
    this.vertexCount = 0;
    this.isALoop = !0;
    this.vertices = [];
  };
  Box2D.inherit(c, Box2D.Collision.Shapes.b2Shape);
  c.prototype.__super = Box2D.Collision.Shapes.b2Shape.prototype;
  c.b2EdgeShape = function () {
    Box2D.Collision.Shapes.b2Shape.b2Shape.apply(this, arguments);
    this.s_supportVec = new G();
    this.m_v1 = new G();
    this.m_v2 = new G();
    this.m_coreV1 = new G();
    this.m_coreV2 = new G();
    this.m_normal = new G();
    this.m_direction = new G();
    this.m_cornerDir1 = new G();
    this.m_cornerDir2 = new G();
  };
  c.prototype.TestPoint = function (e, g) {
    return !1;
  };
  c.prototype.RayCast = function (e, g, c) {
    var u = g.p2.x - g.p1.x,
      l = g.p2.y - g.p1.y;
    var h = c.R;
    var w = c.position.x + (h.col1.x * this.m_v1.x + h.col2.x * this.m_v1.y),
      C = c.position.y + (h.col1.y * this.m_v1.x + h.col2.y * this.m_v1.y),
      n = c.position.y + (h.col1.y * this.m_v2.x + h.col2.y * this.m_v2.y) - C;
    c = -(c.position.x + (h.col1.x * this.m_v2.x + h.col2.x * this.m_v2.y) - w);
    h = 100 * Number.MIN_VALUE;
    var x = -(u * n + l * c);
    if (x > h) {
      var w = g.p1.x - w,
        t = g.p1.y - C,
        C = w * n + t * c;
      if (
        0 <= C &&
        C <= g.maxFraction * x &&
        ((g = -u * t + l * w), -h * x <= g && g <= x * (1 + h))
      )
        return (
          (e.fraction = C / x),
          (g = Math.sqrt(n * n + c * c)),
          (e.normal.x = n / g),
          (e.normal.y = c / g),
          !0
        );
    }
    return !1;
  };
  c.prototype.ComputeAABB = function (e, g) {
    var c = g.R,
      h = g.position.x + (c.col1.x * this.m_v1.x + c.col2.x * this.m_v1.y),
      l = g.position.y + (c.col1.y * this.m_v1.x + c.col2.y * this.m_v1.y),
      u = g.position.x + (c.col1.x * this.m_v2.x + c.col2.x * this.m_v2.y),
      c = g.position.y + (c.col1.y * this.m_v2.x + c.col2.y * this.m_v2.y);
    h < u
      ? ((e.lowerBound.x = h), (e.upperBound.x = u))
      : ((e.lowerBound.x = u), (e.upperBound.x = h));
    l < c
      ? ((e.lowerBound.y = l), (e.upperBound.y = c))
      : ((e.lowerBound.y = c), (e.upperBound.y = l));
  };
  c.prototype.ComputeMass = function (e, g) {
    e.mass = 0;
    e.center.SetV(this.m_v1);
    e.I = 0;
  };
  c.prototype.ComputeSubmergedArea = function (c, g, h, l) {
    void 0 === g && (g = 0);
    var w = new G(c.x * g, c.y * g),
      u = e.MulX(h, this.m_v1);
    h = e.MulX(h, this.m_v2);
    var n = e.Dot(c, u) - g;
    c = e.Dot(c, h) - g;
    if (0 < n) {
      if (0 < c) return 0;
      u.x = (-c / (n - c)) * u.x + (n / (n - c)) * h.x;
      u.y = (-c / (n - c)) * u.y + (n / (n - c)) * h.y;
    } else
      0 < c &&
        ((h.x = (-c / (n - c)) * u.x + (n / (n - c)) * h.x),
        (h.y = (-c / (n - c)) * u.y + (n / (n - c)) * h.y));
    l.x = (w.x + u.x + h.x) / 3;
    l.y = (w.y + u.y + h.y) / 3;
    return 0.5 * ((u.x - w.x) * (h.y - w.y) - (u.y - w.y) * (h.x - w.x));
  };
  c.prototype.GetLength = function () {
    return this.m_length;
  };
  c.prototype.GetVertex1 = function () {
    return this.m_v1;
  };
  c.prototype.GetVertex2 = function () {
    return this.m_v2;
  };
  c.prototype.GetCoreVertex1 = function () {
    return this.m_coreV1;
  };
  c.prototype.GetCoreVertex2 = function () {
    return this.m_coreV2;
  };
  c.prototype.GetNormalVector = function () {
    return this.m_normal;
  };
  c.prototype.GetDirectionVector = function () {
    return this.m_direction;
  };
  c.prototype.GetCorner1Vector = function () {
    return this.m_cornerDir1;
  };
  c.prototype.GetCorner2Vector = function () {
    return this.m_cornerDir2;
  };
  c.prototype.Corner1IsConvex = function () {
    return this.m_cornerConvex1;
  };
  c.prototype.Corner2IsConvex = function () {
    return this.m_cornerConvex2;
  };
  c.prototype.GetFirstVertex = function (e) {
    var g = e.R;
    return new G(
      e.position.x + (g.col1.x * this.m_coreV1.x + g.col2.x * this.m_coreV1.y),
      e.position.y + (g.col1.y * this.m_coreV1.x + g.col2.y * this.m_coreV1.y)
    );
  };
  c.prototype.GetNextEdge = function () {
    return this.m_nextEdge;
  };
  c.prototype.GetPrevEdge = function () {
    return this.m_prevEdge;
  };
  c.prototype.Support = function (e, g, c) {
    void 0 === g && (g = 0);
    void 0 === c && (c = 0);
    var h = e.R,
      l =
        e.position.x +
        (h.col1.x * this.m_coreV1.x + h.col2.x * this.m_coreV1.y),
      w =
        e.position.y +
        (h.col1.y * this.m_coreV1.x + h.col2.y * this.m_coreV1.y),
      u =
        e.position.x +
        (h.col1.x * this.m_coreV2.x + h.col2.x * this.m_coreV2.y);
    e =
      e.position.y + (h.col1.y * this.m_coreV2.x + h.col2.y * this.m_coreV2.y);
    l * g + w * c > u * g + e * c
      ? ((this.s_supportVec.x = l), (this.s_supportVec.y = w))
      : ((this.s_supportVec.x = u), (this.s_supportVec.y = e));
    return this.s_supportVec;
  };
  c.prototype.b2EdgeShape = function (e, g) {
    this.__super.b2Shape.call(this);
    this.m_type = A.e_edgeShape;
    this.m_nextEdge = this.m_prevEdge = null;
    this.m_v1 = e;
    this.m_v2 = g;
    this.m_direction.Set(this.m_v2.x - this.m_v1.x, this.m_v2.y - this.m_v1.y);
    this.m_length = this.m_direction.Normalize();
    this.m_normal.Set(this.m_direction.y, -this.m_direction.x);
    this.m_coreV1.Set(
      -n.b2_toiSlop * (this.m_normal.x - this.m_direction.x) + this.m_v1.x,
      -n.b2_toiSlop * (this.m_normal.y - this.m_direction.y) + this.m_v1.y
    );
    this.m_coreV2.Set(
      -n.b2_toiSlop * (this.m_normal.x + this.m_direction.x) + this.m_v2.x,
      -n.b2_toiSlop * (this.m_normal.y + this.m_direction.y) + this.m_v2.y
    );
    this.m_cornerDir1 = this.m_normal;
    this.m_cornerDir2.Set(-this.m_normal.x, -this.m_normal.y);
  };
  c.prototype.SetPrevEdge = function (e, g, c, h) {
    this.m_prevEdge = e;
    this.m_coreV1 = g;
    this.m_cornerDir1 = c;
    this.m_cornerConvex1 = h;
  };
  c.prototype.SetNextEdge = function (e, g, c, h) {
    this.m_nextEdge = e;
    this.m_coreV2 = g;
    this.m_cornerDir2 = c;
    this.m_cornerConvex2 = h;
  };
  h.b2MassData = function () {
    this.mass = 0;
    this.center = new G(0, 0);
    this.I = 0;
  };
  Box2D.inherit(l, Box2D.Collision.Shapes.b2Shape);
  l.prototype.__super = Box2D.Collision.Shapes.b2Shape.prototype;
  l.b2PolygonShape = function () {
    Box2D.Collision.Shapes.b2Shape.b2Shape.apply(this, arguments);
  };
  l.prototype.Copy = function () {
    var e = new l();
    e.Set(this);
    return e;
  };
  l.prototype.Set = function (e) {
    this.__super.Set.call(this, e);
    if (Box2D.is(e, l)) {
      e = e instanceof l ? e : null;
      this.m_centroid.SetV(e.m_centroid);
      this.m_vertexCount = e.m_vertexCount;
      this.Reserve(this.m_vertexCount);
      for (var g = 0; g < this.m_vertexCount; g++)
        this.m_vertices[g].SetV(e.m_vertices[g]),
          this.m_normals[g].SetV(e.m_normals[g]);
    }
  };
  l.prototype.SetAsArray = function (e, g) {
    void 0 === g && (g = 0);
    var c = new Vector(),
      h;
    for (h = 0; h < e.length; ++h) {
      var l = e[h];
      c.push(l);
    }
    this.SetAsVector(c, g);
  };
  l.AsArray = function (e, g) {
    void 0 === g && (g = 0);
    var c = new l();
    c.SetAsArray(e, g);
    return c;
  };
  l.prototype.SetAsVector = function (c, g) {
    void 0 === g && (g = 0);
    0 == g && (g = c.length);
    n.b2Assert(2 <= g);
    this.m_vertexCount = g;
    this.Reserve(g);
    var h;
    for (h = 0; h < this.m_vertexCount; h++) this.m_vertices[h].SetV(c[h]);
    for (h = 0; h < this.m_vertexCount; ++h) {
      var w = parseInt(h),
        u = parseInt(h + 1 < this.m_vertexCount ? h + 1 : 0),
        w = e.SubtractVV(this.m_vertices[u], this.m_vertices[w]);
      n.b2Assert(w.LengthSquared() > Number.MIN_VALUE);
      this.m_normals[h].SetV(e.CrossVF(w, 1));
      this.m_normals[h].Normalize();
    }
    this.m_centroid = l.ComputeCentroid(this.m_vertices, this.m_vertexCount);
  };
  l.AsVector = function (e, g) {
    void 0 === g && (g = 0);
    var c = new l();
    c.SetAsVector(e, g);
    return c;
  };
  l.prototype.SetAsBox = function (e, g) {
    void 0 === e && (e = 0);
    void 0 === g && (g = 0);
    this.m_vertexCount = 4;
    this.Reserve(4);
    this.m_vertices[0].Set(-e, -g);
    this.m_vertices[1].Set(e, -g);
    this.m_vertices[2].Set(e, g);
    this.m_vertices[3].Set(-e, g);
    this.m_normals[0].Set(0, -1);
    this.m_normals[1].Set(1, 0);
    this.m_normals[2].Set(0, 1);
    this.m_normals[3].Set(-1, 0);
    this.m_centroid.SetZero();
  };
  l.AsBox = function (e, g) {
    void 0 === e && (e = 0);
    void 0 === g && (g = 0);
    var c = new l();
    c.SetAsBox(e, g);
    return c;
  };
  l.prototype.SetAsOrientedBox = function (c, g, h, l) {
    void 0 === c && (c = 0);
    void 0 === g && (g = 0);
    void 0 === h && (h = null);
    void 0 === l && (l = 0);
    this.m_vertexCount = 4;
    this.Reserve(4);
    this.m_vertices[0].Set(-c, -g);
    this.m_vertices[1].Set(c, -g);
    this.m_vertices[2].Set(c, g);
    this.m_vertices[3].Set(-c, g);
    this.m_normals[0].Set(0, -1);
    this.m_normals[1].Set(1, 0);
    this.m_normals[2].Set(0, 1);
    this.m_normals[3].Set(-1, 0);
    this.m_centroid = h;
    c = new w();
    c.position = h;
    c.R.Set(l);
    for (h = 0; h < this.m_vertexCount; ++h)
      (this.m_vertices[h] = e.MulX(c, this.m_vertices[h])),
        (this.m_normals[h] = e.MulMV(c.R, this.m_normals[h]));
  };
  l.AsOrientedBox = function (e, g, c, h) {
    void 0 === e && (e = 0);
    void 0 === g && (g = 0);
    void 0 === c && (c = null);
    void 0 === h && (h = 0);
    var w = new l();
    w.SetAsOrientedBox(e, g, c, h);
    return w;
  };
  l.prototype.SetAsEdge = function (c, g) {
    this.m_vertexCount = 2;
    this.Reserve(2);
    this.m_vertices[0].SetV(c);
    this.m_vertices[1].SetV(g);
    this.m_centroid.x = 0.5 * (c.x + g.x);
    this.m_centroid.y = 0.5 * (c.y + g.y);
    this.m_normals[0] = e.CrossVF(e.SubtractVV(g, c), 1);
    this.m_normals[0].Normalize();
    this.m_normals[1].x = -this.m_normals[0].x;
    this.m_normals[1].y = -this.m_normals[0].y;
  };
  l.AsEdge = function (e, g) {
    var c = new l();
    c.SetAsEdge(e, g);
    return c;
  };
  l.prototype.TestPoint = function (e, g) {
    var c = e.R;
    for (
      var h = g.x - e.position.x,
        l = g.y - e.position.y,
        w = h * c.col1.x + l * c.col1.y,
        n = h * c.col2.x + l * c.col2.y,
        u = 0;
      u < this.m_vertexCount;
      ++u
    )
      if (
        ((c = this.m_vertices[u]),
        (h = w - c.x),
        (l = n - c.y),
        (c = this.m_normals[u]),
        0 < c.x * h + c.y * l)
      )
        return !1;
    return !0;
  };
  l.prototype.RayCast = function (e, g, c) {
    var h = 0,
      l = g.maxFraction;
    var w = g.p1.x - c.position.x;
    var n = g.p1.y - c.position.y;
    var u = c.R;
    var x = w * u.col1.x + n * u.col1.y,
      t = w * u.col2.x + n * u.col2.y;
    w = g.p2.x - c.position.x;
    n = g.p2.y - c.position.y;
    u = c.R;
    g = w * u.col1.x + n * u.col1.y - x;
    u = w * u.col2.x + n * u.col2.y - t;
    for (var y = -1, G = 0; G < this.m_vertexCount; ++G) {
      var C = this.m_vertices[G];
      w = C.x - x;
      n = C.y - t;
      C = this.m_normals[G];
      w = C.x * w + C.y * n;
      n = C.x * g + C.y * u;
      if (0 == n) {
        if (0 > w) return !1;
      } else
        0 > n && w < h * n
          ? ((h = w / n), (y = G))
          : 0 < n && w < l * n && (l = w / n);
      if (l < h - Number.MIN_VALUE) return !1;
    }
    return 0 <= y
      ? ((e.fraction = h),
        (u = c.R),
        (C = this.m_normals[y]),
        (e.normal.x = u.col1.x * C.x + u.col2.x * C.y),
        (e.normal.y = u.col1.y * C.x + u.col2.y * C.y),
        !0)
      : !1;
  };
  l.prototype.ComputeAABB = function (e, g) {
    for (
      var c = g.R,
        h = this.m_vertices[0],
        l = g.position.x + (c.col1.x * h.x + c.col2.x * h.y),
        w = g.position.y + (c.col1.y * h.x + c.col2.y * h.y),
        n = l,
        u = w,
        x = 1;
      x < this.m_vertexCount;
      ++x
    )
      var h = this.m_vertices[x],
        t = g.position.x + (c.col1.x * h.x + c.col2.x * h.y),
        h = g.position.y + (c.col1.y * h.x + c.col2.y * h.y),
        l = l < t ? l : t,
        w = w < h ? w : h,
        n = n > t ? n : t,
        u = u > h ? u : h;
    e.lowerBound.x = l - this.m_radius;
    e.lowerBound.y = w - this.m_radius;
    e.upperBound.x = n + this.m_radius;
    e.upperBound.y = u + this.m_radius;
  };
  l.prototype.ComputeMass = function (e, g) {
    void 0 === g && (g = 0);
    if (2 == this.m_vertexCount)
      (e.center.x = 0.5 * (this.m_vertices[0].x + this.m_vertices[1].x)),
        (e.center.y = 0.5 * (this.m_vertices[0].y + this.m_vertices[1].y)),
        (e.mass = 0),
        (e.I = 0);
    else {
      for (
        var c = 0, h = 0, l = 0, w = 1 / 3, n = 0;
        n < this.m_vertexCount;
        ++n
      )
        var u = this.m_vertices[n],
          x =
            n + 1 < this.m_vertexCount
              ? this.m_vertices[parseInt(n + 1)]
              : this.m_vertices[0],
          t = 0.5 * ((u.x - 0) * (x.y - 0) - (u.y - 0) * (x.x - 0)),
          l = l + t,
          c = c + t * w * (0 + u.x + x.x),
          h = h + t * w * (0 + u.y + x.y);
      e.mass = 5 * g;
      e.center.Set((1 / l) * c, (1 / l) * h);
      e.I = 6;
    }
  };
  l.prototype.ComputeSubmergedArea = function (c, g, l, w) {
    void 0 === g && (g = 0);
    var n = e.MulTMV(l.R, c),
      x = g - e.Dot(c, l.position),
      u = new Vector_a2j_Number(),
      t = 0,
      y = -1;
    g = -1;
    var C = !1;
    for (c = 0; c < this.m_vertexCount; ++c) {
      u[c] = e.Dot(n, this.m_vertices[c]) - x;
      var E = u[c] < -Number.MIN_VALUE;
      0 < c && (E ? C || ((y = c - 1), t++) : C && ((g = c - 1), t++));
      C = E;
    }
    switch (t) {
      case 0:
        return C
          ? ((c = new h()),
            this.ComputeMass(c, 1),
            w.SetV(e.MulX(l, c.center)),
            c.mass)
          : 0;
      case 1:
        -1 == y ? (y = this.m_vertexCount - 1) : (g = this.m_vertexCount - 1);
    }
    c = parseInt((y + 1) % this.m_vertexCount);
    n = parseInt((g + 1) % this.m_vertexCount);
    x = (0 - u[y]) / (u[c] - u[y]);
    u = (0 - u[g]) / (u[n] - u[g]);
    y = new G(
      this.m_vertices[y].x * (1 - x) + this.m_vertices[c].x * x,
      this.m_vertices[y].y * (1 - x) + this.m_vertices[c].y * x
    );
    g = new G(
      this.m_vertices[g].x * (1 - u) + this.m_vertices[n].x * u,
      this.m_vertices[g].y * (1 - u) + this.m_vertices[n].y * u
    );
    u = 0;
    x = new G();
    for (t = this.m_vertices[c]; c != n; )
      (c = (c + 1) % this.m_vertexCount),
        (C = c == n ? g : this.m_vertices[c]),
        (E = 0.5 * ((t.x - y.x) * (C.y - y.y) - (t.y - y.y) * (C.x - y.x))),
        (u += E),
        (x.x += (E * (y.x + t.x + C.x)) / 3),
        (x.y += (E * (y.y + t.y + C.y)) / 3),
        (t = C);
    x.Multiply(1 / u);
    w.SetV(e.MulX(l, x));
    return u;
  };
  l.prototype.GetVertexCount = function () {
    return this.m_vertexCount;
  };
  l.prototype.GetVertices = function () {
    return this.m_vertices;
  };
  l.prototype.GetNormals = function () {
    return this.m_normals;
  };
  l.prototype.GetSupport = function (e) {
    for (
      var g = 0,
        c = this.m_vertices[0].x * e.x + this.m_vertices[0].y * e.y,
        h = 1;
      h < this.m_vertexCount;
      ++h
    ) {
      var l = this.m_vertices[h].x * e.x + this.m_vertices[h].y * e.y;
      l > c && ((g = h), (c = l));
    }
    return g;
  };
  l.prototype.GetSupportVertex = function (e) {
    for (
      var g = 0,
        c = this.m_vertices[0].x * e.x + this.m_vertices[0].y * e.y,
        h = 1;
      h < this.m_vertexCount;
      ++h
    ) {
      var l = this.m_vertices[h].x * e.x + this.m_vertices[h].y * e.y;
      l > c && ((g = h), (c = l));
    }
    return this.m_vertices[g];
  };
  l.prototype.Validate = function () {
    return !1;
  };
  l.prototype.b2PolygonShape = function () {
    this.__super.b2Shape.call(this);
    this.m_type = A.e_polygonShape;
    this.m_centroid = new G();
    this.m_vertices = new Vector();
    this.m_normals = new Vector();
  };
  l.prototype.Reserve = function (e) {
    void 0 === e && (e = 0);
    for (var g = parseInt(this.m_vertices.length); g < e; g++)
      (this.m_vertices[g] = new G()), (this.m_normals[g] = new G());
  };
  l.ComputeCentroid = function (e, g) {
    void 0 === g && (g = 0);
    for (var c = new G(), h = 0, l = 1 / 3, w = 0; w < g; ++w) {
      var n = e[w],
        x = w + 1 < g ? e[parseInt(w + 1)] : e[0],
        u = 0.5 * ((n.x - 0) * (x.y - 0) - (n.y - 0) * (x.x - 0)),
        h = h + u;
      c.x += u * l * (0 + n.x + x.x);
      c.y += u * l * (0 + n.y + x.y);
    }
    c.x *= 1 / h;
    c.y *= 1 / h;
    return c;
  };
  A.b2Shape = function () {};
  A.prototype.Copy = function () {
    return null;
  };
  A.prototype.Set = function (e) {
    this.m_radius = e.m_radius;
  };
  A.prototype.GetType = function () {
    return this.m_type;
  };
  A.prototype.TestPoint = function (e, g) {
    return !1;
  };
  A.prototype.RayCast = function (e, g, c) {
    return !1;
  };
  A.prototype.ComputeAABB = function (e, g) {};
  A.prototype.ComputeMass = function (e, g) {};
  A.prototype.ComputeSubmergedArea = function (e, g, c, h) {
    return 0;
  };
  A.TestOverlap = function (e, g, c, h) {
    var l = new H();
    l.proxyA = new y();
    l.proxyA.Set(e);
    l.proxyB = new y();
    l.proxyB.Set(c);
    l.transformA = g;
    l.transformB = h;
    l.useRadii = !0;
    e = new Q();
    e.count = 0;
    g = new L();
    P.Distance(g, e, l);
    return g.distance < 10 * Number.MIN_VALUE;
  };
  A.prototype.b2Shape = function () {
    this.m_type = A.e_unknownShape;
    this.m_radius = n.b2_linearSlop;
  };
  Box2D.postDefs.push(function () {
    Box2D.Collision.Shapes.b2Shape.e_unknownShape = -1;
    Box2D.Collision.Shapes.b2Shape.e_circleShape = 0;
    Box2D.Collision.Shapes.b2Shape.e_polygonShape = 1;
    Box2D.Collision.Shapes.b2Shape.e_edgeShape = 2;
    Box2D.Collision.Shapes.b2Shape.e_shapeTypeCount = 3;
    Box2D.Collision.Shapes.b2Shape.e_hitCollide = 1;
    Box2D.Collision.Shapes.b2Shape.e_missCollide = 0;
    Box2D.Collision.Shapes.b2Shape.e_startsInsideCollide = -1;
  });
})();
(function () {
  var n = Box2D.Common.b2Color,
    x = Box2D.Common.b2Settings,
    t = Box2D.Common.Math.b2Math;
  n.b2Color = function () {
    this._b = this._g = this._r = 0;
  };
  n.prototype.b2Color = function (c, h, l) {
    void 0 === c && (c = 0);
    void 0 === h && (h = 0);
    void 0 === l && (l = 0);
    this._r = Box2D.parseUInt(255 * t.Clamp(c, 0, 1));
    this._g = Box2D.parseUInt(255 * t.Clamp(h, 0, 1));
    this._b = Box2D.parseUInt(255 * t.Clamp(l, 0, 1));
  };
  n.prototype.Set = function (c, h, l) {
    void 0 === c && (c = 0);
    void 0 === h && (h = 0);
    void 0 === l && (l = 0);
    this._r = Box2D.parseUInt(255 * t.Clamp(c, 0, 1));
    this._g = Box2D.parseUInt(255 * t.Clamp(h, 0, 1));
    this._b = Box2D.parseUInt(255 * t.Clamp(l, 0, 1));
  };
  Object.defineProperty(n.prototype, "r", {
    enumerable: !1,
    configurable: !0,
    set: function (c) {
      void 0 === c && (c = 0);
      this._r = Box2D.parseUInt(255 * t.Clamp(c, 0, 1));
    },
  });
  Object.defineProperty(n.prototype, "g", {
    enumerable: !1,
    configurable: !0,
    set: function (c) {
      void 0 === c && (c = 0);
      this._g = Box2D.parseUInt(255 * t.Clamp(c, 0, 1));
    },
  });
  Object.defineProperty(n.prototype, "b", {
    enumerable: !1,
    configurable: !0,
    set: function (c) {
      void 0 === c && (c = 0);
      this._b = Box2D.parseUInt(255 * t.Clamp(c, 0, 1));
    },
  });
  Object.defineProperty(n.prototype, "color", {
    enumerable: !1,
    configurable: !0,
    get: function () {
      return (this._r << 16) | (this._g << 8) | this._b;
    },
  });
  x.b2Settings = function () {};
  x.b2MixFriction = function (c, h) {
    void 0 === c && (c = 0);
    void 0 === h && (h = 0);
    return Math.sqrt(c * h);
  };
  x.b2MixRestitution = function (c, h) {
    void 0 === c && (c = 0);
    void 0 === h && (h = 0);
    return c > h ? c : h;
  };
  x.b2Assert = function (c) {
    if (!c) throw "Assertion Failed";
  };
  Box2D.postDefs.push(function () {
    Box2D.Common.b2Settings.VERSION = "2.1alpha";
    Box2D.Common.b2Settings.USHRT_MAX = 65535;
    Box2D.Common.b2Settings.b2_pi = Math.PI;
    Box2D.Common.b2Settings.b2_maxManifoldPoints = 2;
    Box2D.Common.b2Settings.b2_aabbExtension = 0.1;
    Box2D.Common.b2Settings.b2_aabbMultiplier = 2;
    Box2D.Common.b2Settings.b2_polygonRadius = 2 * x.b2_linearSlop;
    Box2D.Common.b2Settings.b2_linearSlop = 0.005;
    Box2D.Common.b2Settings.b2_angularSlop = (2 / 180) * x.b2_pi;
    Box2D.Common.b2Settings.b2_toiSlop = 8 * x.b2_linearSlop;
    Box2D.Common.b2Settings.b2_maxTOIContactsPerIsland = 32;
    Box2D.Common.b2Settings.b2_maxTOIJointsPerIsland = 32;
    Box2D.Common.b2Settings.b2_velocityThreshold = 1;
    Box2D.Common.b2Settings.b2_maxLinearCorrection = 0.2;
    Box2D.Common.b2Settings.b2_maxAngularCorrection = (8 / 180) * x.b2_pi;
    Box2D.Common.b2Settings.b2_maxTranslation = 2;
    Box2D.Common.b2Settings.b2_maxTranslationSquared =
      x.b2_maxTranslation * x.b2_maxTranslation;
    Box2D.Common.b2Settings.b2_maxRotation = 0.5 * x.b2_pi;
    Box2D.Common.b2Settings.b2_maxRotationSquared =
      x.b2_maxRotation * x.b2_maxRotation;
    Box2D.Common.b2Settings.b2_contactBaumgarte = 0.2;
    Box2D.Common.b2Settings.b2_timeToSleep = 0.5;
    Box2D.Common.b2Settings.b2_linearSleepTolerance = 0.01;
    Box2D.Common.b2Settings.b2_angularSleepTolerance = (2 / 180) * x.b2_pi;
  });
})();
(function () {
  var n = Box2D.Common.Math.b2Mat22,
    x = Box2D.Common.Math.b2Mat33,
    t = Box2D.Common.Math.b2Math,
    c = Box2D.Common.Math.b2Sweep,
    h = Box2D.Common.Math.b2Transform,
    l = Box2D.Common.Math.b2Vec2,
    A = Box2D.Common.Math.b2Vec3;
  n.b2Mat22 = function () {
    this.col1 = new l();
    this.col2 = new l();
  };
  n.prototype.b2Mat22 = function () {
    this.SetIdentity();
  };
  n.FromAngle = function (e) {
    void 0 === e && (e = 0);
    var c = new n();
    c.Set(e);
    return c;
  };
  n.FromVV = function (e, c) {
    var h = new n();
    h.SetVV(e, c);
    return h;
  };
  n.prototype.Set = function (e) {
    void 0 === e && (e = 0);
    var c = Math.cos(e);
    e = Math.sin(e);
    this.col1.x = c;
    this.col2.x = -e;
    this.col1.y = e;
    this.col2.y = c;
  };
  n.prototype.SetVV = function (e, c) {
    this.col1.SetV(e);
    this.col2.SetV(c);
  };
  n.prototype.Copy = function () {
    var e = new n();
    e.SetM(this);
    return e;
  };
  n.prototype.SetM = function (e) {
    this.col1.SetV(e.col1);
    this.col2.SetV(e.col2);
  };
  n.prototype.AddM = function (e) {
    this.col1.x += e.col1.x;
    this.col1.y += e.col1.y;
    this.col2.x += e.col2.x;
    this.col2.y += e.col2.y;
  };
  n.prototype.SetIdentity = function () {
    this.col1.x = 1;
    this.col2.x = 0;
    this.col1.y = 0;
    this.col2.y = 1;
  };
  n.prototype.SetZero = function () {
    this.col1.x = 0;
    this.col2.x = 0;
    this.col1.y = 0;
    this.col2.y = 0;
  };
  n.prototype.GetAngle = function () {
    return Math.atan2(this.col1.y, this.col1.x);
  };
  n.prototype.GetInverse = function (e) {
    var c = this.col1.x,
      h = this.col2.x,
      l = this.col1.y,
      n = this.col2.y,
      x = c * n - h * l;
    0 != x && (x = 1 / x);
    e.col1.x = x * n;
    e.col2.x = -x * h;
    e.col1.y = -x * l;
    e.col2.y = x * c;
    return e;
  };
  n.prototype.Solve = function (e, c, h) {
    void 0 === c && (c = 0);
    void 0 === h && (h = 0);
    var l = this.col1.x,
      w = this.col2.x,
      n = this.col1.y,
      x = this.col2.y,
      t = l * x - w * n;
    0 != t && (t = 1 / t);
    e.x = t * (x * c - w * h);
    e.y = t * (l * h - n * c);
    return e;
  };
  n.prototype.Abs = function () {
    this.col1.Abs();
    this.col2.Abs();
  };
  x.b2Mat33 = function () {
    this.col1 = new A();
    this.col2 = new A();
    this.col3 = new A();
  };
  x.prototype.b2Mat33 = function (e, c, h) {
    void 0 === e && (e = null);
    void 0 === c && (c = null);
    void 0 === h && (h = null);
    e || c || h
      ? (this.col1.SetV(e), this.col2.SetV(c), this.col3.SetV(h))
      : (this.col1.SetZero(), this.col2.SetZero(), this.col3.SetZero());
  };
  x.prototype.SetVVV = function (e, c, h) {
    this.col1.SetV(e);
    this.col2.SetV(c);
    this.col3.SetV(h);
  };
  x.prototype.Copy = function () {
    return new x(this.col1, this.col2, this.col3);
  };
  x.prototype.SetM = function (e) {
    this.col1.SetV(e.col1);
    this.col2.SetV(e.col2);
    this.col3.SetV(e.col3);
  };
  x.prototype.AddM = function (e) {
    this.col1.x += e.col1.x;
    this.col1.y += e.col1.y;
    this.col1.z += e.col1.z;
    this.col2.x += e.col2.x;
    this.col2.y += e.col2.y;
    this.col2.z += e.col2.z;
    this.col3.x += e.col3.x;
    this.col3.y += e.col3.y;
    this.col3.z += e.col3.z;
  };
  x.prototype.SetIdentity = function () {
    this.col1.x = 1;
    this.col2.x = 0;
    this.col3.x = 0;
    this.col1.y = 0;
    this.col2.y = 1;
    this.col3.y = 0;
    this.col1.z = 0;
    this.col2.z = 0;
    this.col3.z = 1;
  };
  x.prototype.SetZero = function () {
    this.col1.x = 0;
    this.col2.x = 0;
    this.col3.x = 0;
    this.col1.y = 0;
    this.col2.y = 0;
    this.col3.y = 0;
    this.col1.z = 0;
    this.col2.z = 0;
    this.col3.z = 0;
  };
  x.prototype.Solve22 = function (e, c, h) {
    void 0 === c && (c = 0);
    void 0 === h && (h = 0);
    var l = this.col1.x,
      w = this.col2.x,
      n = this.col1.y,
      x = this.col2.y,
      t = l * x - w * n;
    0 != t && (t = 1 / t);
    e.x = t * (x * c - w * h);
    e.y = t * (l * h - n * c);
    return e;
  };
  x.prototype.Solve33 = function (e, c, h, l) {
    void 0 === c && (c = 0);
    void 0 === h && (h = 0);
    void 0 === l && (l = 0);
    var w = this.col1.x,
      n = this.col1.y,
      x = this.col1.z,
      t = this.col2.x,
      u = this.col2.y,
      g = this.col2.z,
      G = this.col3.x,
      A = this.col3.y,
      M = this.col3.z,
      K = w * (u * M - g * A) + n * (g * G - t * M) + x * (t * A - u * G);
    0 != K && (K = 1 / K);
    e.x = K * (c * (u * M - g * A) + h * (g * G - t * M) + l * (t * A - u * G));
    e.y = K * (w * (h * M - l * A) + n * (l * G - c * M) + x * (c * A - h * G));
    e.z = K * (w * (u * l - g * h) + n * (g * c - t * l) + x * (t * h - u * c));
    return e;
  };
  t.b2Math = function () {};
  t.IsValid = function (e) {
    void 0 === e && (e = 0);
    return isFinite(e);
  };
  t.Dot = function (e, c) {
    return e.x * c.x + e.y * c.y;
  };
  t.CrossVV = function (e, c) {
    return e.x * c.y - e.y * c.x;
  };
  t.CrossVF = function (e, c) {
    void 0 === c && (c = 0);
    return new l(c * e.y, -c * e.x);
  };
  t.CrossFV = function (e, c) {
    void 0 === e && (e = 0);
    return new l(-e * c.y, e * c.x);
  };
  t.MulMV = function (e, c) {
    return new l(
      e.col1.x * c.x + e.col2.x * c.y,
      e.col1.y * c.x + e.col2.y * c.y
    );
  };
  t.MulTMV = function (e, c) {
    return new l(t.Dot(c, e.col1), t.Dot(c, e.col2));
  };
  t.MulX = function (e, c) {
    var h = t.MulMV(e.R, c);
    h.x += e.position.x;
    h.y += e.position.y;
    return h;
  };
  t.MulXT = function (e, c) {
    var h = t.SubtractVV(c, e.position),
      l = h.x * e.R.col1.x + h.y * e.R.col1.y;
    h.y = h.x * e.R.col2.x + h.y * e.R.col2.y;
    h.x = l;
    return h;
  };
  t.AddVV = function (e, c) {
    return new l(e.x + c.x, e.y + c.y);
  };
  t.SubtractVV = function (e, c) {
    return new l(e.x - c.x, e.y - c.y);
  };
  t.Distance = function (e, c) {
    var h = e.x - c.x,
      l = e.y - c.y;
    return Math.sqrt(h * h + l * l);
  };
  t.DistanceSquared = function (e, c) {
    var h = e.x - c.x,
      l = e.y - c.y;
    return h * h + l * l;
  };
  t.MulFV = function (e, c) {
    void 0 === e && (e = 0);
    return new l(e * c.x, e * c.y);
  };
  t.AddMM = function (e, c) {
    return n.FromVV(t.AddVV(e.col1, c.col1), t.AddVV(e.col2, c.col2));
  };
  t.MulMM = function (e, c) {
    return n.FromVV(t.MulMV(e, c.col1), t.MulMV(e, c.col2));
  };
  t.MulTMM = function (e, c) {
    var h = new l(t.Dot(e.col1, c.col1), t.Dot(e.col2, c.col1)),
      w = new l(t.Dot(e.col1, c.col2), t.Dot(e.col2, c.col2));
    return n.FromVV(h, w);
  };
  t.Abs = function (e) {
    void 0 === e && (e = 0);
    return 0 < e ? e : -e;
  };
  t.AbsV = function (e) {
    return new l(t.Abs(e.x), t.Abs(e.y));
  };
  t.AbsM = function (e) {
    return n.FromVV(t.AbsV(e.col1), t.AbsV(e.col2));
  };
  t.Min = function (e, c) {
    void 0 === e && (e = 0);
    void 0 === c && (c = 0);
    return e < c ? e : c;
  };
  t.MinV = function (e, c) {
    return new l(t.Min(e.x, c.x), t.Min(e.y, c.y));
  };
  t.Max = function (e, c) {
    void 0 === e && (e = 0);
    void 0 === c && (c = 0);
    return e > c ? e : c;
  };
  t.MaxV = function (e, c) {
    return new l(t.Max(e.x, c.x), t.Max(e.y, c.y));
  };
  t.Clamp = function (e, c, h) {
    void 0 === e && (e = 0);
    void 0 === c && (c = 0);
    void 0 === h && (h = 0);
    return e < c ? c : e > h ? h : e;
  };
  t.ClampV = function (e, c, h) {
    return t.MaxV(c, t.MinV(e, h));
  };
  t.Swap = function (e, c) {
    var h = e[0];
    e[0] = c[0];
    c[0] = h;
  };
  t.Random = function () {
    return 2 * Math.random() - 1;
  };
  t.RandomRange = function (e, c) {
    void 0 === e && (e = 0);
    void 0 === c && (c = 0);
    return (c - e) * Math.random() + e;
  };
  t.NextPowerOfTwo = function (e) {
    void 0 === e && (e = 0);
    e |= (e >> 1) & 2147483647;
    e |= (e >> 2) & 1073741823;
    e |= (e >> 4) & 268435455;
    e |= (e >> 8) & 16777215;
    return (e | ((e >> 16) & 65535)) + 1;
  };
  t.IsPowerOfTwo = function (e) {
    void 0 === e && (e = 0);
    return 0 < e && 0 == (e & (e - 1));
  };
  Box2D.postDefs.push(function () {
    Box2D.Common.Math.b2Math.b2Vec2_zero = new l(0, 0);
    Box2D.Common.Math.b2Math.b2Mat22_identity = n.FromVV(
      new l(1, 0),
      new l(0, 1)
    );
    Box2D.Common.Math.b2Math.b2Transform_identity = new h(
      t.b2Vec2_zero,
      t.b2Mat22_identity
    );
  });
  c.b2Sweep = function () {
    this.localCenter = new l();
    this.c0 = new l();
    this.c = new l();
  };
  c.prototype.Set = function (e) {
    this.localCenter.SetV(e.localCenter);
    this.c0.SetV(e.c0);
    this.c.SetV(e.c);
    this.a0 = e.a0;
    this.a = e.a;
    this.t0 = e.t0;
  };
  c.prototype.Copy = function () {
    var e = new c();
    e.localCenter.SetV(this.localCenter);
    e.c0.SetV(this.c0);
    e.c.SetV(this.c);
    e.a0 = this.a0;
    e.a = this.a;
    e.t0 = this.t0;
    return e;
  };
  c.prototype.GetTransform = function (e, c) {
    void 0 === c && (c = 0);
    e.position.x = (1 - c) * this.c0.x + c * this.c.x;
    e.position.y = (1 - c) * this.c0.y + c * this.c.y;
    e.R.Set((1 - c) * this.a0 + c * this.a);
    var h = e.R;
    e.position.x -=
      h.col1.x * this.localCenter.x + h.col2.x * this.localCenter.y;
    e.position.y -=
      h.col1.y * this.localCenter.x + h.col2.y * this.localCenter.y;
  };
  c.prototype.Advance = function (e) {
    void 0 === e && (e = 0);
    if (this.t0 < e && 1 - this.t0 > Number.MIN_VALUE) {
      var c = (e - this.t0) / (1 - this.t0);
      this.c0.x = (1 - c) * this.c0.x + c * this.c.x;
      this.c0.y = (1 - c) * this.c0.y + c * this.c.y;
      this.a0 = (1 - c) * this.a0 + c * this.a;
      this.t0 = e;
    }
  };
  h.b2Transform = function () {
    this.position = new l();
    this.R = new n();
  };
  h.prototype.b2Transform = function (e, c) {
    void 0 === e && (e = null);
    void 0 === c && (c = null);
    e && (this.position.SetV(e), this.R.SetM(c));
  };
  h.prototype.Initialize = function (e, c) {
    this.position.SetV(e);
    this.R.SetM(c);
  };
  h.prototype.SetIdentity = function () {
    this.position.SetZero();
    this.R.SetIdentity();
  };
  h.prototype.Set = function (e) {
    this.position.SetV(e.position);
    this.R.SetM(e.R);
  };
  h.prototype.GetAngle = function () {
    return Math.atan2(this.R.col1.y, this.R.col1.x);
  };
  l.b2Vec2 = function () {};
  l.prototype.b2Vec2 = function (e, c) {
    void 0 === e && (e = 0);
    void 0 === c && (c = 0);
    this.x = e;
    this.y = c;
  };
  l.prototype.SetZero = function () {
    this.y = this.x = 0;
  };
  l.prototype.Set = function (e, c) {
    void 0 === e && (e = 0);
    void 0 === c && (c = 0);
    this.x = e;
    this.y = c;
  };
  l.prototype.SetV = function (c) {
    this.x = c.x;
    this.y = c.y;
  };
  l.prototype.GetNegative = function () {
    return new l(-this.x, -this.y);
  };
  l.prototype.NegativeSelf = function () {
    this.x = -this.x;
    this.y = -this.y;
  };
  l.Make = function (c, h) {
    void 0 === c && (c = 0);
    void 0 === h && (h = 0);
    return new l(c, h);
  };
  l.prototype.Copy = function () {
    return new l(this.x, this.y);
  };
  l.prototype.Add = function (c) {
    this.x += c.x;
    this.y += c.y;
  };
  l.prototype.Subtract = function (c) {
    this.x -= c.x;
    this.y -= c.y;
  };
  l.prototype.Multiply = function (c) {
    void 0 === c && (c = 0);
    this.x *= c;
    this.y *= c;
  };
  l.prototype.MulM = function (c) {
    var e = this.x;
    this.x = c.col1.x * e + c.col2.x * this.y;
    this.y = c.col1.y * e + c.col2.y * this.y;
  };
  l.prototype.MulTM = function (c) {
    var e = t.Dot(this, c.col1);
    this.y = t.Dot(this, c.col2);
    this.x = e;
  };
  l.prototype.CrossVF = function (c) {
    void 0 === c && (c = 0);
    var e = this.x;
    this.x = c * this.y;
    this.y = -c * e;
  };
  l.prototype.CrossFV = function (c) {
    void 0 === c && (c = 0);
    var e = this.x;
    this.x = -c * this.y;
    this.y = c * e;
  };
  l.prototype.MinV = function (c) {
    this.x = this.x < c.x ? this.x : c.x;
    this.y = this.y < c.y ? this.y : c.y;
  };
  l.prototype.MaxV = function (c) {
    this.x = this.x > c.x ? this.x : c.x;
    this.y = this.y > c.y ? this.y : c.y;
  };
  l.prototype.Abs = function () {
    0 > this.x && (this.x = -this.x);
    0 > this.y && (this.y = -this.y);
  };
  l.prototype.Length = function () {
    return Math.sqrt(this.x * this.x + this.y * this.y);
  };
  l.prototype.LengthSquared = function () {
    return this.x * this.x + this.y * this.y;
  };
  l.prototype.Normalize = function () {
    var c = Math.sqrt(this.x * this.x + this.y * this.y);
    if (c < Number.MIN_VALUE) return 0;
    var h = 1 / c;
    this.x *= h;
    this.y *= h;
    return c;
  };
  l.prototype.IsValid = function () {
    return t.IsValid(this.x) && t.IsValid(this.y);
  };
  A.b2Vec3 = function () {};
  A.prototype.b2Vec3 = function (c, h, l) {
    void 0 === c && (c = 0);
    void 0 === h && (h = 0);
    void 0 === l && (l = 0);
    this.x = c;
    this.y = h;
    this.z = l;
  };
  A.prototype.SetZero = function () {
    this.x = this.y = this.z = 0;
  };
  A.prototype.Set = function (c, h, l) {
    void 0 === c && (c = 0);
    void 0 === h && (h = 0);
    void 0 === l && (l = 0);
    this.x = c;
    this.y = h;
    this.z = l;
  };
  A.prototype.SetV = function (c) {
    this.x = c.x;
    this.y = c.y;
    this.z = c.z;
  };
  A.prototype.GetNegative = function () {
    return new A(-this.x, -this.y, -this.z);
  };
  A.prototype.NegativeSelf = function () {
    this.x = -this.x;
    this.y = -this.y;
    this.z = -this.z;
  };
  A.prototype.Copy = function () {
    return new A(this.x, this.y, this.z);
  };
  A.prototype.Add = function (c) {
    this.x += c.x;
    this.y += c.y;
    this.z += c.z;
  };
  A.prototype.Subtract = function (c) {
    this.x -= c.x;
    this.y -= c.y;
    this.z -= c.z;
  };
  A.prototype.Multiply = function (c) {
    void 0 === c && (c = 0);
    this.x *= c;
    this.y *= c;
    this.z *= c;
  };
})();
(function () {
  var n = Box2D.Common.Math.b2Math,
    x = Box2D.Common.Math.b2Sweep,
    t = Box2D.Common.Math.b2Transform,
    c = Box2D.Common.Math.b2Vec2,
    h = Box2D.Common.b2Color,
    l = Box2D.Common.b2Settings,
    A = Box2D.Collision.b2AABB,
    e = Box2D.Collision.b2ContactPoint,
    w = Box2D.Collision.b2DynamicTreeBroadPhase,
    G = Box2D.Collision.b2RayCastInput,
    P = Box2D.Collision.b2RayCastOutput,
    H = Box2D.Collision.Shapes.b2CircleShape,
    L = Box2D.Collision.Shapes.b2EdgeShape,
    y = Box2D.Collision.Shapes.b2MassData,
    Q = Box2D.Collision.Shapes.b2PolygonShape,
    u = Box2D.Collision.Shapes.b2Shape,
    g = Box2D.Dynamics.b2Body,
    C = Box2D.Dynamics.b2BodyDef,
    E = Box2D.Dynamics.b2ContactFilter,
    M = Box2D.Dynamics.b2ContactImpulse,
    K = Box2D.Dynamics.b2ContactListener,
    N = Box2D.Dynamics.b2ContactManager,
    D = Box2D.Dynamics.b2DebugDraw,
    R = Box2D.Dynamics.b2DestructionListener,
    F = Box2D.Dynamics.b2FilterData,
    I = Box2D.Dynamics.b2Fixture,
    S = Box2D.Dynamics.b2FixtureDef,
    O = Box2D.Dynamics.b2Island,
    a = Box2D.Dynamics.b2TimeStep,
    d = Box2D.Dynamics.b2World,
    z = Box2D.Dynamics.Contacts.b2Contact,
    f = Box2D.Dynamics.Contacts.b2ContactFactory,
    r = Box2D.Dynamics.Contacts.b2ContactSolver,
    v = Box2D.Dynamics.Joints.b2Joint,
    J = Box2D.Dynamics.Joints.b2PulleyJoint;
  g.b2Body = function () {
    this.m_xf = new t();
    this.m_sweep = new x();
    this.m_linearVelocity = new c();
    this.m_force = new c();
  };
  g.prototype.connectEdges = function (b, k, a) {
    void 0 === a && (a = 0);
    var d = Math.atan2(k.GetDirectionVector().y, k.GetDirectionVector().x);
    a = n.MulFV(Math.tan(0.5 * (d - a)), k.GetDirectionVector());
    a = n.SubtractVV(a, k.GetNormalVector());
    a = n.MulFV(l.b2_toiSlop, a);
    a = n.AddVV(a, k.GetVertex1());
    var q = n.AddVV(b.GetDirectionVector(), k.GetDirectionVector());
    q.Normalize();
    var f = 0 < n.Dot(b.GetDirectionVector(), k.GetNormalVector());
    b.SetNextEdge(k, a, q, f);
    k.SetPrevEdge(b, a, q, f);
    return d;
  };
  g.prototype.CreateFixture = function (b) {
    if (1 == this.m_world.IsLocked()) return null;
    var k = new I();
    k.Create(this, this.m_xf, b);
    this.m_flags & g.e_activeFlag &&
      k.CreateProxy(this.m_world.m_contactManager.m_broadPhase, this.m_xf);
    k.m_next = this.m_fixtureList;
    this.m_fixtureList = k;
    ++this.m_fixtureCount;
    k.m_body = this;
    0 < k.m_density && this.ResetMassData();
    this.m_world.m_flags |= d.e_newFixture;
    return k;
  };
  g.prototype.CreateFixture2 = function (b, k) {
    void 0 === k && (k = 0);
    var a = new S();
    a.shape = b;
    a.density = k;
    return this.CreateFixture(a);
  };
  g.prototype.DestroyFixture = function (b) {
    if (1 != this.m_world.IsLocked()) {
      for (var a = this.m_fixtureList, d = null; null != a; ) {
        if (a == b) {
          d ? (d.m_next = b.m_next) : (this.m_fixtureList = b.m_next);
          break;
        }
        d = a;
        a = a.m_next;
      }
      for (a = this.m_contactList; a; ) {
        var d = a.contact,
          a = a.next,
          f = d.GetFixtureA(),
          c = d.GetFixtureB();
        (b != f && b != c) || this.m_world.m_contactManager.Destroy(d);
      }
      this.m_flags & g.e_activeFlag &&
        b.DestroyProxy(this.m_world.m_contactManager.m_broadPhase);
      b.Destroy();
      b.m_body = null;
      b.m_next = null;
      --this.m_fixtureCount;
      this.ResetMassData();
    }
  };
  g.prototype.SetPositionAndAngle = function (b, a) {
    void 0 === a && (a = 0);
    if (1 != this.m_world.IsLocked()) {
      this.m_xf.R.Set(a);
      this.m_xf.position.SetV(b);
      var k = this.m_xf.R;
      var d = this.m_sweep.localCenter;
      this.m_sweep.c.x = k.col1.x * d.x + k.col2.x * d.y;
      this.m_sweep.c.y = k.col1.y * d.x + k.col2.y * d.y;
      this.m_sweep.c.x += this.m_xf.position.x;
      this.m_sweep.c.y += this.m_xf.position.y;
      this.m_sweep.c0.SetV(this.m_sweep.c);
      this.m_sweep.a0 = this.m_sweep.a = a;
      d = this.m_world.m_contactManager.m_broadPhase;
      for (k = this.m_fixtureList; k; k = k.m_next)
        k.Synchronize(d, this.m_xf, this.m_xf);
      this.m_world.m_contactManager.FindNewContacts();
    }
  };
  g.prototype.SetTransform = function (b) {
    this.SetPositionAndAngle(b.position, b.GetAngle());
  };
  g.prototype.GetTransform = function () {
    return this.m_xf;
  };
  g.prototype.GetPosition = function () {
    return this.m_xf.position;
  };
  g.prototype.SetPosition = function (b) {
    this.SetPositionAndAngle(b, this.GetAngle());
  };
  g.prototype.GetAngle = function () {
    return this.m_sweep.a;
  };
  g.prototype.SetAngle = function (b) {
    void 0 === b && (b = 0);
    this.SetPositionAndAngle(this.GetPosition(), b);
  };
  g.prototype.GetWorldCenter = function () {
    return this.m_sweep.c;
  };
  g.prototype.GetLocalCenter = function () {
    return this.m_sweep.localCenter;
  };
  g.prototype.SetLinearVelocity = function (b) {
    this.m_type != g.b2_staticBody && this.m_linearVelocity.SetV(b);
  };
  g.prototype.GetLinearVelocity = function () {
    return this.m_linearVelocity;
  };
  g.prototype.SetAngularVelocity = function (b) {
    void 0 === b && (b = 0);
    this.m_type != g.b2_staticBody && (this.m_angularVelocity = b);
  };
  g.prototype.GetAngularVelocity = function () {
    return this.m_angularVelocity;
  };
  g.prototype.GetDefinition = function () {
    var b = new C();
    b.type = this.GetType();
    b.allowSleep = (this.m_flags & g.e_allowSleepFlag) == g.e_allowSleepFlag;
    b.angle = this.GetAngle();
    b.angularDamping = this.m_angularDamping;
    b.angularVelocity = this.m_angularVelocity;
    b.fixedRotation =
      (this.m_flags & g.e_fixedRotationFlag) == g.e_fixedRotationFlag;
    b.bullet = (this.m_flags & g.e_bulletFlag) == g.e_bulletFlag;
    b.awake = (this.m_flags & g.e_awakeFlag) == g.e_awakeFlag;
    b.linearDamping = this.m_linearDamping;
    b.linearVelocity.SetV(this.GetLinearVelocity());
    b.position = this.GetPosition();
    b.userData = this.GetUserData();
    return b;
  };
  g.prototype.ApplyForce = function (b, a) {
    this.m_type == g.b2_dynamicBody &&
      (0 == this.IsAwake() && this.SetAwake(!0),
      (this.m_force.x += b.x),
      (this.m_force.y += b.y),
      (this.m_torque +=
        (a.x - this.m_sweep.c.x) * b.y - (a.y - this.m_sweep.c.y) * b.x));
  };
  g.prototype.ApplyTorque = function (b) {
    void 0 === b && (b = 0);
    this.m_type == g.b2_dynamicBody &&
      (0 == this.IsAwake() && this.SetAwake(!0), (this.m_torque += b));
  };
  g.prototype.ApplyImpulse = function (b, a) {
    this.m_type == g.b2_dynamicBody &&
      (0 == this.IsAwake() && this.SetAwake(!0),
      (this.m_linearVelocity.x += this.m_invMass * b.x),
      (this.m_linearVelocity.y += this.m_invMass * b.y),
      (this.m_angularVelocity +=
        this.m_invI *
        ((a.x - this.m_sweep.c.x) * b.y - (a.y - this.m_sweep.c.y) * b.x)));
  };
  g.prototype.Split = function (b) {
    for (
      var a = this.GetLinearVelocity().Copy(),
        d = this.GetAngularVelocity(),
        f = this.GetWorldCenter(),
        c = this.m_world.CreateBody(this.GetDefinition()),
        r,
        v = this.m_fixtureList;
      v;

    )
      if (b(v)) {
        var e = v.m_next;
        r ? (r.m_next = e) : (this.m_fixtureList = e);
        this.m_fixtureCount--;
        v.m_next = c.m_fixtureList;
        c.m_fixtureList = v;
        c.m_fixtureCount++;
        v.m_body = c;
        v = e;
      } else (r = v), (v = v.m_next);
    this.ResetMassData();
    c.ResetMassData();
    r = this.GetWorldCenter();
    b = c.GetWorldCenter();
    r = n.AddVV(a, n.CrossFV(d, n.SubtractVV(r, f)));
    a = n.AddVV(a, n.CrossFV(d, n.SubtractVV(b, f)));
    this.SetLinearVelocity(r);
    c.SetLinearVelocity(a);
    this.SetAngularVelocity(d);
    c.SetAngularVelocity(d);
    this.SynchronizeFixtures();
    c.SynchronizeFixtures();
    return c;
  };
  g.prototype.Merge = function (b) {
    var a;
    for (a = b.m_fixtureList; a; ) {
      var d = a.m_next;
      b.m_fixtureCount--;
      a.m_next = this.m_fixtureList;
      this.m_fixtureList = a;
      this.m_fixtureCount++;
      a.m_body = c;
      a = d;
    }
    f.m_fixtureCount = 0;
    var f = this,
      c = b;
    f.GetWorldCenter();
    c.GetWorldCenter();
    f.GetLinearVelocity().Copy();
    c.GetLinearVelocity().Copy();
    f.GetAngularVelocity();
    c.GetAngularVelocity();
    f.ResetMassData();
    this.SynchronizeFixtures();
  };
  g.prototype.GetMass = function () {
    return this.m_mass;
  };
  g.prototype.GetInertia = function () {
    return this.m_I;
  };
  g.prototype.GetMassData = function (b) {
    b.mass = this.m_mass;
    b.I = this.m_I;
    b.center.SetV(this.m_sweep.localCenter);
  };
  g.prototype.SetMassData = function (b) {
    l.b2Assert(0 == this.m_world.IsLocked());
    if (1 != this.m_world.IsLocked() && this.m_type == g.b2_dynamicBody) {
      this.m_invI = this.m_I = this.m_invMass = 0;
      this.m_mass = b.mass;
      0 >= this.m_mass && (this.m_mass = 1);
      this.m_invMass = 1 / this.m_mass;
      0 < b.I &&
        0 == (this.m_flags & g.e_fixedRotationFlag) &&
        ((this.m_I =
          b.I -
          this.m_mass * (b.center.x * b.center.x + b.center.y * b.center.y)),
        (this.m_invI = 1 / this.m_I));
      var a = this.m_sweep.c.Copy();
      this.m_sweep.localCenter.SetV(b.center);
      this.m_sweep.c0.SetV(n.MulX(this.m_xf, this.m_sweep.localCenter));
      this.m_sweep.c.SetV(this.m_sweep.c0);
      this.m_linearVelocity.x +=
        this.m_angularVelocity * -(this.m_sweep.c.y - a.y);
      this.m_linearVelocity.y +=
        this.m_angularVelocity * +(this.m_sweep.c.x - a.x);
    }
  };
  g.prototype.ResetMassData = function () {
    this.m_invI = this.m_I = this.m_invMass = this.m_mass = 0;
    this.m_sweep.localCenter.SetZero();
    if (this.m_type != g.b2_staticBody && this.m_type != g.b2_kinematicBody) {
      for (var b = c.Make(0, 0), a = this.m_fixtureList; a; a = a.m_next)
        if (0 != a.m_density) {
          var d = a.GetMassData();
          this.m_mass += d.mass;
          b.x += d.center.x * d.mass;
          b.y += d.center.y * d.mass;
          this.m_I += d.I;
        }
      0 < this.m_mass
        ? ((this.m_invMass = 1 / this.m_mass),
          (b.x *= this.m_invMass),
          (b.y *= this.m_invMass))
        : (this.m_invMass = this.m_mass = 1);
      0 < this.m_I && 0 == (this.m_flags & g.e_fixedRotationFlag)
        ? ((this.m_I -= this.m_mass * (b.x * b.x + b.y * b.y)),
          (this.m_I *= this.m_inertiaScale),
          l.b2Assert(0 < this.m_I),
          (this.m_invI = 1 / this.m_I))
        : (this.m_invI = this.m_I = 0);
      a = this.m_sweep.c.Copy();
      this.m_sweep.localCenter.SetV(b);
      this.m_sweep.c0.SetV(n.MulX(this.m_xf, this.m_sweep.localCenter));
      this.m_sweep.c.SetV(this.m_sweep.c0);
      this.m_linearVelocity.x +=
        this.m_angularVelocity * -(this.m_sweep.c.y - a.y);
      this.m_linearVelocity.y +=
        this.m_angularVelocity * +(this.m_sweep.c.x - a.x);
    }
  };
  g.prototype.GetWorldPoint = function (b) {
    var a = this.m_xf.R;
    b = new c(a.col1.x * b.x + a.col2.x * b.y, a.col1.y * b.x + a.col2.y * b.y);
    b.x += this.m_xf.position.x;
    b.y += this.m_xf.position.y;
    return b;
  };
  g.prototype.GetWorldVector = function (b) {
    return n.MulMV(this.m_xf.R, b);
  };
  g.prototype.GetLocalPoint = function (b) {
    return n.MulXT(this.m_xf, b);
  };
  g.prototype.GetLocalVector = function (b) {
    return n.MulTMV(this.m_xf.R, b);
  };
  g.prototype.GetLinearVelocityFromWorldPoint = function (b) {
    return new c(
      this.m_linearVelocity.x -
        this.m_angularVelocity * (b.y - this.m_sweep.c.y),
      this.m_linearVelocity.y +
        this.m_angularVelocity * (b.x - this.m_sweep.c.x)
    );
  };
  g.prototype.GetLinearVelocityFromLocalPoint = function (b) {
    var a = this.m_xf.R;
    b = new c(a.col1.x * b.x + a.col2.x * b.y, a.col1.y * b.x + a.col2.y * b.y);
    b.x += this.m_xf.position.x;
    b.y += this.m_xf.position.y;
    return new c(
      this.m_linearVelocity.x -
        this.m_angularVelocity * (b.y - this.m_sweep.c.y),
      this.m_linearVelocity.y +
        this.m_angularVelocity * (b.x - this.m_sweep.c.x)
    );
  };
  g.prototype.GetLinearDamping = function () {
    return this.m_linearDamping;
  };
  g.prototype.SetLinearDamping = function (b) {
    void 0 === b && (b = 0);
    this.m_linearDamping = b;
  };
  g.prototype.GetAngularDamping = function () {
    return this.m_angularDamping;
  };
  g.prototype.SetAngularDamping = function (b) {
    void 0 === b && (b = 0);
    this.m_angularDamping = b;
  };
  g.prototype.SetType = function (b) {
    void 0 === b && (b = 0);
    if (this.m_type != b)
      for (
        this.m_type = b,
          this.ResetMassData(),
          this.m_type == g.b2_staticBody &&
            (this.m_linearVelocity.SetZero(), (this.m_angularVelocity = 0)),
          this.SetAwake(!0),
          this.m_force.SetZero(),
          this.m_torque = 0,
          b = this.m_contactList;
        b;
        b = b.next
      )
        b.contact.FlagForFiltering();
  };
  g.prototype.GetType = function () {
    return this.m_type;
  };
  g.prototype.SetBullet = function (b) {
    this.m_flags = b
      ? this.m_flags | g.e_bulletFlag
      : this.m_flags & ~g.e_bulletFlag;
  };
  g.prototype.IsBullet = function () {
    return (this.m_flags & g.e_bulletFlag) == g.e_bulletFlag;
  };
  g.prototype.SetSelect = function (b) {
    this.m_select = b;
  };
  g.prototype.IsSelect = function () {
    return this.m_select;
  };
  g.prototype.SetID = function (b) {
    this.m_id = b;
  };
  g.prototype.GetID = function () {
    return this.m_id;
  };
  g.prototype.SetSleepingAllowed = function (b) {
    b
      ? (this.m_flags |= g.e_allowSleepFlag)
      : ((this.m_flags &= ~g.e_allowSleepFlag), this.SetAwake(!0));
  };
  g.prototype.SetAwake = function (b) {
    b
      ? ((this.m_flags |= g.e_awakeFlag), (this.m_sleepTime = 0))
      : ((this.m_flags &= ~g.e_awakeFlag),
        (this.m_sleepTime = 0),
        this.m_linearVelocity.SetZero(),
        (this.m_angularVelocity = 0),
        this.m_force.SetZero(),
        (this.m_torque = 0));
  };
  g.prototype.IsAwake = function () {
    return (this.m_flags & g.e_awakeFlag) == g.e_awakeFlag;
  };
  g.prototype.SetFixedRotation = function (b) {
    this.m_flags = b
      ? this.m_flags | g.e_fixedRotationFlag
      : this.m_flags & ~g.e_fixedRotationFlag;
    this.ResetMassData();
  };
  g.prototype.IsFixedRotation = function () {
    return (this.m_flags & g.e_fixedRotationFlag) == g.e_fixedRotationFlag;
  };
  g.prototype.SetActive = function (b) {
    if (b != this.IsActive()) {
      var a;
      if (b)
        for (
          this.m_flags |= g.e_activeFlag,
            b = this.m_world.m_contactManager.m_broadPhase,
            a = this.m_fixtureList;
          a;
          a = a.m_next
        )
          a.CreateProxy(b, this.m_xf);
      else {
        this.m_flags &= ~g.e_activeFlag;
        b = this.m_world.m_contactManager.m_broadPhase;
        for (a = this.m_fixtureList; a; a = a.m_next) a.DestroyProxy(b);
        for (b = this.m_contactList; b; )
          (a = b),
            (b = b.next),
            this.m_world.m_contactManager.Destroy(a.contact);
        this.m_contactList = null;
      }
    }
  };
  g.prototype.IsActive = function () {
    return (this.m_flags & g.e_activeFlag) == g.e_activeFlag;
  };
  g.prototype.IsSleepingAllowed = function () {
    return (this.m_flags & g.e_allowSleepFlag) == g.e_allowSleepFlag;
  };
  g.prototype.GetFixtureList = function () {
    return this.m_fixtureList;
  };
  g.prototype.GetJointList = function () {
    return this.m_jointList;
  };
  g.prototype.GetControllerList = function () {
    return this.m_controllerList;
  };
  g.prototype.GetContactList = function () {
    return this.m_contactList;
  };
  g.prototype.GetNext = function () {
    return this.m_next;
  };
  g.prototype.GetUserData = function () {
    return this.m_userData;
  };
  g.prototype.SetUserData = function (b) {
    this.m_userData = b;
  };
  g.prototype.GetWorld = function () {
    return this.m_world;
  };
  g.prototype.b2Body = function (b, a) {
    this.m_flags = 0;
    b.bullet && (this.m_flags |= g.e_bulletFlag);
    b.fixedRotation && (this.m_flags |= g.e_fixedRotationFlag);
    b.allowSleep && (this.m_flags |= g.e_allowSleepFlag);
    b.awake && (this.m_flags |= g.e_awakeFlag);
    b.active && (this.m_flags |= g.e_activeFlag);
    this.m_select = b.isSelect;
    this.m_id = -10;
    this.m_world = a;
    this.m_xf.position.SetV(b.position);
    this.m_xf.R.Set(b.angle);
    this.m_sweep.localCenter.SetZero();
    this.m_sweep.t0 = 1;
    this.m_sweep.a0 = this.m_sweep.a = b.angle;
    var k = this.m_xf.R,
      d = this.m_sweep.localCenter;
    this.m_sweep.c.x = k.col1.x * d.x + k.col2.x * d.y;
    this.m_sweep.c.y = k.col1.y * d.x + k.col2.y * d.y;
    this.m_sweep.c.x += this.m_xf.position.x;
    this.m_sweep.c.y += this.m_xf.position.y;
    this.m_sweep.c0.SetV(this.m_sweep.c);
    this.m_contactList = this.m_controllerList = this.m_jointList = null;
    this.m_controllerCount = 0;
    this.m_next = this.m_prev = null;
    this.m_linearVelocity.SetV(b.linearVelocity);
    this.m_angularVelocity = b.angularVelocity;
    this.m_linearDamping = b.linearDamping;
    this.m_angularDamping = b.angularDamping;
    this.m_force.Set(0, 0);
    this.m_sleepTime = this.m_torque = 0;
    this.m_type = b.type;
    this.m_invMass =
      this.m_type == g.b2_dynamicBody ? (this.m_mass = 1) : (this.m_mass = 0);
    this.m_invI = this.m_I = 0;
    this.m_inertiaScale = b.inertiaScale;
    this.m_userData = b.userData;
    this.m_fixtureList = null;
    this.m_fixtureCount = 0;
  };
  g.prototype.SynchronizeFixtures = function () {
    var b = g.s_xf1;
    b.R.Set(this.m_sweep.a0);
    var a = b.R,
      d = this.m_sweep.localCenter;
    b.position.x = this.m_sweep.c0.x - (a.col1.x * d.x + a.col2.x * d.y);
    b.position.y = this.m_sweep.c0.y - (a.col1.y * d.x + a.col2.y * d.y);
    d = this.m_world.m_contactManager.m_broadPhase;
    for (a = this.m_fixtureList; a; a = a.m_next)
      a.Synchronize(d, b, this.m_xf);
  };
  g.prototype.SynchronizeTransform = function () {
    this.m_xf.R.Set(this.m_sweep.a);
    var b = this.m_xf.R,
      a = this.m_sweep.localCenter;
    this.m_xf.position.x = this.m_sweep.c.x - (b.col1.x * a.x + b.col2.x * a.y);
    this.m_xf.position.y = this.m_sweep.c.y - (b.col1.y * a.x + b.col2.y * a.y);
  };
  g.prototype.ShouldCollide = function (b) {
    if (this.m_type != g.b2_dynamicBody && b.m_type != g.b2_dynamicBody)
      return !1;
    for (var a = this.m_jointList; a; a = a.next)
      if (a.other == b && 0 == a.joint.m_collideConnected) return !1;
    return !0;
  };
  g.prototype.Advance = function (b) {
    void 0 === b && (b = 0);
    this.m_sweep.Advance(b);
    this.m_sweep.c.SetV(this.m_sweep.c0);
    this.m_sweep.a = this.m_sweep.a0;
    this.SynchronizeTransform();
  };
  Box2D.postDefs.push(function () {
    Box2D.Dynamics.b2Body.s_xf1 = new t();
    Box2D.Dynamics.b2Body.e_islandFlag = 1;
    Box2D.Dynamics.b2Body.e_awakeFlag = 2;
    Box2D.Dynamics.b2Body.e_allowSleepFlag = 4;
    Box2D.Dynamics.b2Body.e_bulletFlag = 8;
    Box2D.Dynamics.b2Body.e_fixedRotationFlag = 16;
    Box2D.Dynamics.b2Body.e_activeFlag = 32;
    Box2D.Dynamics.b2Body.b2_staticBody = 0;
    Box2D.Dynamics.b2Body.b2_kinematicBody = 1;
    Box2D.Dynamics.b2Body.b2_dynamicBody = 2;
  });
  C.b2BodyDef = function () {
    this.position = new c();
    this.linearVelocity = new c();
  };
  C.prototype.b2BodyDef = function () {
    this.userData = null;
    this.position.Set(0, 0);
    this.angle = 0;
    this.linearVelocity.Set(0, 0);
    this.angularDamping = this.linearDamping = this.angularVelocity = 0;
    this.awake = this.allowSleep = !0;
    this.bullet = this.fixedRotation = !1;
    this.type = g.b2_staticBody;
    this.active = !0;
    this.inertiaScale = 1;
    this.isSelect = !1;
    this.id = -1;
  };
  E.b2ContactFilter = function () {};
  E.prototype.ShouldCollide = function (b, a) {
    var k = b.GetFilterData(),
      d = a.GetFilterData();
    return (
      0 != (k.maskBits & d.categoryBits) && 0 != (k.categoryBits & d.maskBits)
    );
  };
  E.prototype.RayCollide = function (b, a) {
    return b ? this.ShouldCollide(b instanceof I ? b : null, a) : !0;
  };
  Box2D.postDefs.push(function () {
    Box2D.Dynamics.b2ContactFilter.b2_defaultFilter = new E();
  });
  M.b2ContactImpulse = function () {
    this.normalImpulses = new Vector_a2j_Number(l.b2_maxManifoldPoints);
    this.tangentImpulses = new Vector_a2j_Number(l.b2_maxManifoldPoints);
  };
  K.b2ContactListener = function () {};
  K.prototype.BeginContact = function (b) {};
  K.prototype.EndContact = function (b) {};
  K.prototype.PreSolve = function (b, a) {};
  K.prototype.PostSolve = function (b, a) {};
  Box2D.postDefs.push(function () {
    Box2D.Dynamics.b2ContactListener.b2_defaultListener = new K();
  });
  N.b2ContactManager = function () {};
  N.prototype.b2ContactManager = function () {
    this.m_world = null;
    this.m_contactCount = 0;
    this.m_contactFilter = E.b2_defaultFilter;
    this.m_contactListener = K.b2_defaultListener;
    this.m_contactFactory = new f(this.m_allocator);
    this.m_broadPhase = new w();
  };
  N.prototype.AddPair = function (b, a) {
    var k = b instanceof I ? b : null,
      d = a instanceof I ? a : null,
      f = k.GetBody(),
      c = d.GetBody();
    if (f != c) {
      for (var r = c.GetContactList(); r; ) {
        if (r.other == f) {
          var v = r.contact.GetFixtureA(),
            e = r.contact.GetFixtureB();
          if ((v == k && e == d) || (v == d && e == k)) return;
        }
        r = r.next;
      }
      0 != c.ShouldCollide(f) &&
        0 != this.m_contactFilter.ShouldCollide(k, d) &&
        ((r = this.m_contactFactory.Create(k, d)),
        (k = r.GetFixtureA()),
        (d = r.GetFixtureB()),
        (f = k.m_body),
        (c = d.m_body),
        (r.m_prev = null),
        (r.m_next = this.m_world.m_contactList),
        null != this.m_world.m_contactList &&
          (this.m_world.m_contactList.m_prev = r),
        (this.m_world.m_contactList = r),
        (r.m_nodeA.contact = r),
        (r.m_nodeA.other = c),
        (r.m_nodeA.prev = null),
        (r.m_nodeA.next = f.m_contactList),
        null != f.m_contactList && (f.m_contactList.prev = r.m_nodeA),
        (f.m_contactList = r.m_nodeA),
        (r.m_nodeB.contact = r),
        (r.m_nodeB.other = f),
        (r.m_nodeB.prev = null),
        (r.m_nodeB.next = c.m_contactList),
        null != c.m_contactList && (c.m_contactList.prev = r.m_nodeB),
        (c.m_contactList = r.m_nodeB),
        ++this.m_world.m_contactCount);
    }
  };
  N.prototype.FindNewContacts = function () {
    this.m_broadPhase.UpdatePairs(Box2D.generateCallback(this, this.AddPair));
  };
  N.prototype.Destroy = function (b) {
    var a = b.GetFixtureA(),
      d = b.GetFixtureB(),
      a = a.GetBody(),
      d = d.GetBody();
    b.IsTouching() && this.m_contactListener.EndContact(b);
    b.m_prev && (b.m_prev.m_next = b.m_next);
    b.m_next && (b.m_next.m_prev = b.m_prev);
    b == this.m_world.m_contactList && (this.m_world.m_contactList = b.m_next);
    b.m_nodeA.prev && (b.m_nodeA.prev.next = b.m_nodeA.next);
    b.m_nodeA.next && (b.m_nodeA.next.prev = b.m_nodeA.prev);
    b.m_nodeA == a.m_contactList && (a.m_contactList = b.m_nodeA.next);
    b.m_nodeB.prev && (b.m_nodeB.prev.next = b.m_nodeB.next);
    b.m_nodeB.next && (b.m_nodeB.next.prev = b.m_nodeB.prev);
    b.m_nodeB == d.m_contactList && (d.m_contactList = b.m_nodeB.next);
    this.m_contactFactory.Destroy(b);
    --this.m_contactCount;
  };
  N.prototype.Collide = function () {
    for (var b, a = this.m_world.m_contactList; a; ) {
      b = a.GetFixtureA();
      var d = a.GetFixtureB(),
        f = b.GetBody(),
        c = d.GetBody();
      if (0 == f.IsAwake() && 0 == c.IsAwake()) a = a.GetNext();
      else {
        if (a.m_flags & z.e_filterFlag) {
          if (0 == c.ShouldCollide(f)) {
            b = a;
            a = b.GetNext();
            this.Destroy(b);
            continue;
          }
          if (0 == this.m_contactFilter.ShouldCollide(b, d)) {
            b = a;
            a = b.GetNext();
            this.Destroy(b);
            continue;
          }
          a.m_flags &= ~z.e_filterFlag;
        }
        0 == this.m_broadPhase.TestOverlap(b.m_proxy, d.m_proxy)
          ? ((b = a), (a = b.GetNext()), this.Destroy(b))
          : (a.Update(this.m_contactListener), (a = a.GetNext()));
      }
    }
  };
  Box2D.postDefs.push(function () {
    Box2D.Dynamics.b2ContactManager.s_evalCP = new e();
  });
  D.b2DebugDraw = function () {};
  D.prototype.b2DebugDraw = function () {};
  D.prototype.SetFlags = function (b) {};
  D.prototype.GetFlags = function () {};
  D.prototype.AppendFlags = function (b) {};
  D.prototype.ClearFlags = function (b) {};
  D.prototype.SetSprite = function (b) {};
  D.prototype.GetSprite = function () {};
  D.prototype.SetDrawScale = function (b) {};
  D.prototype.GetDrawScale = function () {};
  D.prototype.SetLineThickness = function (b) {};
  D.prototype.GetLineThickness = function () {};
  D.prototype.SetAlpha = function (b) {};
  D.prototype.GetAlpha = function () {};
  D.prototype.SetFillAlpha = function (b) {};
  D.prototype.GetFillAlpha = function () {};
  D.prototype.SetXFormScale = function (b) {};
  D.prototype.GetXFormScale = function () {};
  D.prototype.DrawPolygon = function (b, a, d) {};
  D.prototype.DrawSolidPolygon = function (b, a, d) {};
  D.prototype.DrawCircle = function (b, a, d) {};
  D.prototype.DrawSolidCircle = function (b, a, d, f) {};
  D.prototype.DrawSegment = function (b, a, d) {};
  D.prototype.DrawTransform = function (b) {};
  Box2D.postDefs.push(function () {
    Box2D.Dynamics.b2DebugDraw.e_shapeBit = 1;
    Box2D.Dynamics.b2DebugDraw.e_jointBit = 2;
    Box2D.Dynamics.b2DebugDraw.e_aabbBit = 4;
    Box2D.Dynamics.b2DebugDraw.e_pairBit = 8;
    Box2D.Dynamics.b2DebugDraw.e_centerOfMassBit = 16;
    Box2D.Dynamics.b2DebugDraw.e_controllerBit = 32;
  });
  R.b2DestructionListener = function () {};
  R.prototype.SayGoodbyeJoint = function (b) {};
  R.prototype.SayGoodbyeFixture = function (b) {};
  F.b2FilterData = function () {
    this.categoryBits = 1;
    this.maskBits = 65535;
    this.groupIndex = 0;
  };
  F.prototype.Copy = function () {
    var b = new F();
    b.categoryBits = this.categoryBits;
    b.maskBits = this.maskBits;
    b.groupIndex = this.groupIndex;
    return b;
  };
  I.b2Fixture = function () {
    this.m_filter = new F();
  };
  I.prototype.GetType = function () {
    return this.m_shape.GetType();
  };
  I.prototype.GetShape = function () {
    return this.m_shape;
  };
  I.prototype.SetSensor = function (b) {
    if (this.m_isSensor != b && ((this.m_isSensor = b), null != this.m_body))
      for (b = this.m_body.GetContactList(); b; ) {
        var a = b.contact,
          d = a.GetFixtureA(),
          f = a.GetFixtureB();
        (d != this && f != this) || a.SetSensor(d.IsSensor() || f.IsSensor());
        b = b.next;
      }
  };
  I.prototype.IsSensor = function () {
    return this.m_isSensor;
  };
  I.prototype.SetFilterData = function (b) {
    this.m_filter = b.Copy();
    if (!this.m_body)
      for (b = this.m_body.GetContactList(); b; ) {
        var a = b.contact,
          d = a.GetFixtureA(),
          f = a.GetFixtureB();
        (d != this && f != this) || a.FlagForFiltering();
        b = b.next;
      }
  };
  I.prototype.GetFilterData = function () {
    return this.m_filter.Copy();
  };
  I.prototype.GetBody = function () {
    return this.m_body;
  };
  I.prototype.GetNext = function () {
    return this.m_next;
  };
  I.prototype.GetUserData = function () {
    return this.m_userData;
  };
  I.prototype.SetUserData = function (b) {
    this.m_userData = b;
  };
  I.prototype.TestPoint = function (b) {
    return this.m_shape.TestPoint(this.m_body.GetTransform(), b);
  };
  I.prototype.RayCast = function (b, a) {
    return this.m_shape.RayCast(b, a, this.m_body.GetTransform());
  };
  I.prototype.GetMassData = function (b) {
    void 0 === b && (b = null);
    null == b && (b = new y());
    this.m_shape.ComputeMass(b, this.m_density);
    return b;
  };
  I.prototype.SetDensity = function (b) {
    void 0 === b && (b = 0);
    this.m_density = b;
  };
  I.prototype.GetDensity = function () {
    return this.m_density;
  };
  I.prototype.GetFriction = function () {
    return this.m_friction;
  };
  I.prototype.SetFriction = function (b) {
    void 0 === b && (b = 0);
    this.m_friction = b;
  };
  I.prototype.GetRestitution = function () {
    return this.m_restitution;
  };
  I.prototype.SetRestitution = function (b) {
    void 0 === b && (b = 0);
    this.m_restitution = b;
  };
  I.prototype.GetAABB = function () {
    return this.m_aabb;
  };
  I.prototype.b2Fixture = function () {
    this.m_aabb = new A();
    this.m_shape = this.m_next = this.m_body = this.m_userData = null;
    this.m_restitution = this.m_friction = this.m_density = 0;
  };
  I.prototype.Create = function (b, a, d) {
    this.m_userData = d.userData;
    this.m_friction = d.friction;
    this.m_restitution = d.restitution;
    this.m_body = b;
    this.m_next = null;
    this.m_filter = d.filter.Copy();
    this.m_isSensor = d.isSensor;
    this.m_shape = d.shape.Copy();
    this.m_density = d.density;
  };
  I.prototype.Destroy = function () {
    this.m_shape = null;
  };
  I.prototype.CreateProxy = function (b, a) {
    this.m_shape.ComputeAABB(this.m_aabb, a);
    this.m_proxy = b.CreateProxy(this.m_aabb, this);
  };
  I.prototype.DestroyProxy = function (b) {
    null != this.m_proxy &&
      (b.DestroyProxy(this.m_proxy), (this.m_proxy = null));
  };
  I.prototype.Synchronize = function (b, a, d) {
    if (this.m_proxy) {
      var k = new A(),
        f = new A();
      this.m_shape.ComputeAABB(k, a);
      this.m_shape.ComputeAABB(f, d);
      this.m_aabb.Combine(k, f);
      a = n.SubtractVV(d.position, a.position);
      b.MoveProxy(this.m_proxy, this.m_aabb, a);
    }
  };
  S.b2FixtureDef = function () {
    this.filter = new F();
  };
  S.prototype.b2FixtureDef = function () {
    this.userData = this.shape = null;
    this.friction = 0.2;
    this.density = this.restitution = 0;
    this.filter.categoryBits = 1;
    this.filter.maskBits = 65535;
    this.filter.groupIndex = 0;
    this.isSensor = !1;
  };
  O.b2Island = function () {};
  O.prototype.b2Island = function () {
    this.m_bodies = new Vector();
    this.m_contacts = new Vector();
    this.m_joints = new Vector();
  };
  O.prototype.Initialize = function (b, a, d, f, c, r) {
    void 0 === b && (b = 0);
    void 0 === a && (a = 0);
    void 0 === d && (d = 0);
    this.m_bodyCapacity = b;
    this.m_contactCapacity = a;
    this.m_jointCapacity = d;
    this.m_jointCount = this.m_contactCount = this.m_bodyCount = 0;
    this.m_allocator = f;
    this.m_listener = c;
    this.m_contactSolver = r;
    for (f = this.m_bodies.length; f < b; f++) this.m_bodies[f] = null;
    for (f = this.m_contacts.length; f < a; f++) this.m_contacts[f] = null;
    for (f = this.m_joints.length; f < d; f++) this.m_joints[f] = null;
  };
  O.prototype.Clear = function () {
    this.m_jointCount = this.m_contactCount = this.m_bodyCount = 0;
  };
  O.prototype.Solve = function (b, a, d) {
    var k;
    for (k = 0; k < this.m_bodyCount; ++k) {
      var f = this.m_bodies[k];
      f.GetType() == g.b2_dynamicBody &&
        ((f.m_linearVelocity.x += b.dt * (a.x + f.m_invMass * f.m_force.x)),
        (f.m_linearVelocity.y += b.dt * (a.y + f.m_invMass * f.m_force.y)),
        (f.m_angularVelocity += b.dt * f.m_invI * f.m_torque),
        f.m_linearVelocity.Multiply(
          n.Clamp(1 - b.dt * f.m_linearDamping, 0, 1)
        ),
        (f.m_angularVelocity *= n.Clamp(1 - b.dt * f.m_angularDamping, 0, 1)));
    }
    this.m_contactSolver.Initialize(
      b,
      this.m_contacts,
      this.m_contactCount,
      this.m_allocator
    );
    a = this.m_contactSolver;
    a.InitVelocityConstraints(b);
    for (k = 0; k < this.m_jointCount; ++k) {
      var c = this.m_joints[k];
      c.InitVelocityConstraints(b);
    }
    for (k = 0; k < b.velocityIterations; ++k) {
      for (f = 0; f < this.m_jointCount; ++f)
        (c = this.m_joints[f]), c.SolveVelocityConstraints(b);
      a.SolveVelocityConstraints();
    }
    for (k = 0; k < this.m_jointCount; ++k)
      (c = this.m_joints[k]), c.FinalizeVelocityConstraints();
    a.FinalizeVelocityConstraints();
    for (k = 0; k < this.m_bodyCount; ++k)
      if (((f = this.m_bodies[k]), f.GetType() != g.b2_staticBody)) {
        var r = b.dt * f.m_linearVelocity.x,
          q = b.dt * f.m_linearVelocity.y;
        r * r + q * q > l.b2_maxTranslationSquared &&
          (f.m_linearVelocity.Normalize(),
          (f.m_linearVelocity.x =
            f.m_linearVelocity.x * l.b2_maxTranslation * b.inv_dt),
          (f.m_linearVelocity.y =
            f.m_linearVelocity.y * l.b2_maxTranslation * b.inv_dt));
        r = b.dt * f.m_angularVelocity;
        r * r > l.b2_maxRotationSquared &&
          (f.m_angularVelocity =
            0 > f.m_angularVelocity
              ? -l.b2_maxRotation * b.inv_dt
              : l.b2_maxRotation * b.inv_dt);
        f.m_sweep.c0.SetV(f.m_sweep.c);
        f.m_sweep.a0 = f.m_sweep.a;
        f.m_sweep.c.x += b.dt * f.m_linearVelocity.x;
        f.m_sweep.c.y += b.dt * f.m_linearVelocity.y;
        f.m_sweep.a += b.dt * f.m_angularVelocity;
        f.SynchronizeTransform();
      }
    for (k = 0; k < b.positionIterations; ++k) {
      r = a.SolvePositionConstraints(l.b2_contactBaumgarte);
      q = !0;
      for (f = 0; f < this.m_jointCount; ++f)
        (c = this.m_joints[f]),
          (c = c.SolvePositionConstraints(l.b2_contactBaumgarte)),
          (q = q && c);
      if (r && q) break;
    }
    this.Report(a.m_constraints);
    if (d) {
      d = Number.MAX_VALUE;
      a = l.b2_linearSleepTolerance * l.b2_linearSleepTolerance;
      r = l.b2_angularSleepTolerance * l.b2_angularSleepTolerance;
      for (k = 0; k < this.m_bodyCount; ++k)
        (f = this.m_bodies[k]),
          f.GetType() != g.b2_staticBody &&
            (0 == (f.m_flags & g.e_allowSleepFlag) && (d = f.m_sleepTime = 0),
            0 == (f.m_flags & g.e_allowSleepFlag) ||
            f.m_angularVelocity * f.m_angularVelocity > r ||
            n.Dot(f.m_linearVelocity, f.m_linearVelocity) > a
              ? (d = f.m_sleepTime = 0)
              : ((f.m_sleepTime += b.dt), (d = n.Min(d, f.m_sleepTime))));
      if (d >= l.b2_timeToSleep)
        for (k = 0; k < this.m_bodyCount; ++k)
          (f = this.m_bodies[k]), f.SetAwake(!1);
    }
  };
  O.prototype.SolveTOI = function (b) {
    var a, d;
    this.m_contactSolver.Initialize(
      b,
      this.m_contacts,
      this.m_contactCount,
      this.m_allocator
    );
    var f = this.m_contactSolver;
    for (a = 0; a < this.m_jointCount; ++a)
      this.m_joints[a].InitVelocityConstraints(b);
    for (a = 0; a < b.velocityIterations; ++a)
      for (f.SolveVelocityConstraints(), d = 0; d < this.m_jointCount; ++d)
        this.m_joints[d].SolveVelocityConstraints(b);
    for (a = 0; a < this.m_bodyCount; ++a)
      if (((d = this.m_bodies[a]), d.GetType() != g.b2_staticBody)) {
        var c = b.dt * d.m_linearVelocity.x,
          r = b.dt * d.m_linearVelocity.y;
        c * c + r * r > l.b2_maxTranslationSquared &&
          (d.m_linearVelocity.Normalize(),
          (d.m_linearVelocity.x =
            d.m_linearVelocity.x * l.b2_maxTranslation * b.inv_dt),
          (d.m_linearVelocity.y =
            d.m_linearVelocity.y * l.b2_maxTranslation * b.inv_dt));
        c = b.dt * d.m_angularVelocity;
        c * c > l.b2_maxRotationSquared &&
          (d.m_angularVelocity =
            0 > d.m_angularVelocity
              ? -l.b2_maxRotation * b.inv_dt
              : l.b2_maxRotation * b.inv_dt);
        d.m_sweep.c0.SetV(d.m_sweep.c);
        d.m_sweep.a0 = d.m_sweep.a;
        d.m_sweep.c.x += b.dt * d.m_linearVelocity.x;
        d.m_sweep.c.y += b.dt * d.m_linearVelocity.y;
        d.m_sweep.a += b.dt * d.m_angularVelocity;
        d.SynchronizeTransform();
      }
    for (a = 0; a < b.positionIterations; ++a) {
      c = f.SolvePositionConstraints(0.75);
      r = !0;
      for (d = 0; d < this.m_jointCount; ++d)
        var v = this.m_joints[d].SolvePositionConstraints(
            l.b2_contactBaumgarte
          ),
          r = r && v;
      if (c && r) break;
    }
    this.Report(f.m_constraints);
  };
  O.prototype.Report = function (b) {
    if (null != this.m_listener)
      for (var a = 0; a < this.m_contactCount; ++a) {
        for (var d = this.m_contacts[a], f = b[a], c = 0; c < f.pointCount; ++c)
          (O.s_impulse.normalImpulses[c] = f.points[c].normalImpulse),
            (O.s_impulse.tangentImpulses[c] = f.points[c].tangentImpulse);
        this.m_listener.PostSolve(d, O.s_impulse);
      }
  };
  O.prototype.AddBody = function (b) {
    b.m_islandIndex = this.m_bodyCount;
    this.m_bodies[this.m_bodyCount++] = b;
  };
  O.prototype.AddContact = function (b) {
    this.m_contacts[this.m_contactCount++] = b;
  };
  O.prototype.AddJoint = function (b) {
    this.m_joints[this.m_jointCount++] = b;
  };
  Box2D.postDefs.push(function () {
    Box2D.Dynamics.b2Island.s_impulse = new M();
  });
  a.b2TimeStep = function () {};
  a.prototype.Set = function (b) {
    this.dt = b.dt;
    this.inv_dt = b.inv_dt;
    this.positionIterations = b.positionIterations;
    this.velocityIterations = b.velocityIterations;
    this.warmStarting = b.warmStarting;
  };
  d.b2World = function () {
    this.s_stack = new Vector();
    this.m_contactManager = new N();
    this.m_contactSolver = new r();
    this.m_island = new O();
  };
  d.prototype.b2World = function (b, a) {
    this.m_controllerList =
      this.m_jointList =
      this.m_contactList =
      this.m_bodyList =
      this.m_debugDraw =
      this.m_destructionListener =
        null;
    this.m_controllerCount =
      this.m_jointCount =
      this.m_contactCount =
      this.m_bodyCount =
        0;
    d.m_warmStarting = !0;
    d.m_continuousPhysics = !0;
    this.m_allowSleep = a;
    this.m_gravity = b;
    this.m_inv_dt0 = 0;
    this.m_contactManager.m_world = this;
    var k = new C();
    this.m_groundBody = this.CreateBody(k);
  };
  d.prototype.SetDestructionListener = function (b) {
    this.m_destructionListener = b;
  };
  d.prototype.SetContactFilter = function (b) {
    this.m_contactManager.m_contactFilter = b;
  };
  d.prototype.SetContactListener = function (b) {
    this.m_contactManager.m_contactListener = b;
  };
  d.prototype.SetDebugDraw = function (b) {
    this.m_debugDraw = b;
  };
  d.prototype.SetBroadPhase = function (b) {
    var a = this.m_contactManager.m_broadPhase;
    this.m_contactManager.m_broadPhase = b;
    for (var d = this.m_bodyList; d; d = d.m_next)
      for (var f = d.m_fixtureList; f; f = f.m_next)
        f.m_proxy = b.CreateProxy(a.GetFatAABB(f.m_proxy), f);
  };
  d.prototype.Validate = function () {
    this.m_contactManager.m_broadPhase.Validate();
  };
  d.prototype.GetProxyCount = function () {
    return this.m_contactManager.m_broadPhase.GetProxyCount();
  };
  d.prototype.CreateBody = function (b) {
    if (1 == this.IsLocked()) return null;
    b = new g(b, this);
    b.m_prev = null;
    if ((b.m_next = this.m_bodyList)) this.m_bodyList.m_prev = b;
    this.m_bodyList = b;
    ++this.m_bodyCount;
    return b;
  };
  d.prototype.DestroyBody = function (b) {
    if (1 != this.IsLocked()) {
      for (var a = b.m_jointList; a; ) {
        var d = a,
          a = a.next;
        this.m_destructionListener &&
          this.m_destructionListener.SayGoodbyeJoint(d.joint);
        this.DestroyJoint(d.joint);
      }
      for (a = b.m_controllerList; a; )
        (d = a), (a = a.nextController), d.controller.RemoveBody(b);
      for (a = b.m_contactList; a; )
        (d = a), (a = a.next), this.m_contactManager.Destroy(d.contact);
      b.m_contactList = null;
      for (a = b.m_fixtureList; a; )
        (d = a),
          (a = a.m_next),
          this.m_destructionListener &&
            this.m_destructionListener.SayGoodbyeFixture(d),
          d.DestroyProxy(this.m_contactManager.m_broadPhase),
          d.Destroy();
      b.m_fixtureList = null;
      b.m_fixtureCount = 0;
      b.m_prev && (b.m_prev.m_next = b.m_next);
      b.m_next && (b.m_next.m_prev = b.m_prev);
      b == this.m_bodyList && (this.m_bodyList = b.m_next);
      --this.m_bodyCount;
    }
  };
  d.prototype.CreateJoint = function (b) {
    var a = v.Create(b, null);
    a.m_prev = null;
    if ((a.m_next = this.m_jointList)) this.m_jointList.m_prev = a;
    this.m_jointList = a;
    ++this.m_jointCount;
    a.m_edgeA.joint = a;
    a.m_edgeA.other = a.m_bodyB;
    a.m_edgeA.prev = null;
    if ((a.m_edgeA.next = a.m_bodyA.m_jointList))
      a.m_bodyA.m_jointList.prev = a.m_edgeA;
    a.m_bodyA.m_jointList = a.m_edgeA;
    a.m_edgeB.joint = a;
    a.m_edgeB.other = a.m_bodyA;
    a.m_edgeB.prev = null;
    if ((a.m_edgeB.next = a.m_bodyB.m_jointList))
      a.m_bodyB.m_jointList.prev = a.m_edgeB;
    a.m_bodyB.m_jointList = a.m_edgeB;
    var d = b.bodyA,
      f = b.bodyB;
    if (0 == b.collideConnected)
      for (b = f.GetContactList(); b; )
        b.other == d && b.contact.FlagForFiltering(), (b = b.next);
    return a;
  };
  d.prototype.DestroyJoint = function (b) {
    var a = b.m_collideConnected;
    b.m_prev && (b.m_prev.m_next = b.m_next);
    b.m_next && (b.m_next.m_prev = b.m_prev);
    b == this.m_jointList && (this.m_jointList = b.m_next);
    var d = b.m_bodyA,
      f = b.m_bodyB;
    d.SetAwake(!0);
    f.SetAwake(!0);
    b.m_edgeA.prev && (b.m_edgeA.prev.next = b.m_edgeA.next);
    b.m_edgeA.next && (b.m_edgeA.next.prev = b.m_edgeA.prev);
    b.m_edgeA == d.m_jointList && (d.m_jointList = b.m_edgeA.next);
    b.m_edgeA.prev = null;
    b.m_edgeA.next = null;
    b.m_edgeB.prev && (b.m_edgeB.prev.next = b.m_edgeB.next);
    b.m_edgeB.next && (b.m_edgeB.next.prev = b.m_edgeB.prev);
    b.m_edgeB == f.m_jointList && (f.m_jointList = b.m_edgeB.next);
    b.m_edgeB.prev = null;
    b.m_edgeB.next = null;
    v.Destroy(b, null);
    --this.m_jointCount;
    if (0 == a)
      for (b = f.GetContactList(); b; )
        b.other == d && b.contact.FlagForFiltering(), (b = b.next);
  };
  d.prototype.AddController = function (b) {
    b.m_next = this.m_controllerList;
    b.m_prev = null;
    this.m_controllerList = b;
    b.m_world = this;
    this.m_controllerCount++;
    return b;
  };
  d.prototype.RemoveController = function (b) {
    b.m_prev && (b.m_prev.m_next = b.m_next);
    b.m_next && (b.m_next.m_prev = b.m_prev);
    this.m_controllerList == b && (this.m_controllerList = b.m_next);
    this.m_controllerCount--;
  };
  d.prototype.CreateController = function (b) {
    if (b.m_world != this)
      throw Error("Controller can only be a member of one world");
    b.m_next = this.m_controllerList;
    b.m_prev = null;
    this.m_controllerList && (this.m_controllerList.m_prev = b);
    this.m_controllerList = b;
    ++this.m_controllerCount;
    b.m_world = this;
    return b;
  };
  d.prototype.DestroyController = function (b) {
    b.Clear();
    b.m_next && (b.m_next.m_prev = b.m_prev);
    b.m_prev && (b.m_prev.m_next = b.m_next);
    b == this.m_controllerList && (this.m_controllerList = b.m_next);
    --this.m_controllerCount;
  };
  d.prototype.SetWarmStarting = function (b) {
    d.m_warmStarting = b;
  };
  d.prototype.SetContinuousPhysics = function (b) {
    d.m_continuousPhysics = b;
  };
  d.prototype.GetBodyCount = function () {
    return this.m_bodyCount;
  };
  d.prototype.GetJointCount = function () {
    return this.m_jointCount;
  };
  d.prototype.GetContactCount = function () {
    return this.m_contactCount;
  };
  d.prototype.SetGravity = function (b) {
    this.m_gravity = b;
  };
  d.prototype.GetGravity = function () {
    return this.m_gravity;
  };
  d.prototype.GetGroundBody = function () {
    return this.m_groundBody;
  };
  d.prototype.Step = function (b, a, f) {
    void 0 === b && (b = 0);
    void 0 === a && (a = 0);
    void 0 === f && (f = 0);
    this.m_flags & d.e_newFixture &&
      (this.m_contactManager.FindNewContacts(),
      (this.m_flags &= ~d.e_newFixture));
    this.m_flags |= d.e_locked;
    var k = d.s_timestep2;
    k.dt = b;
    k.velocityIterations = a;
    k.positionIterations = f;
    k.inv_dt = 0 < b ? 1 / b : 0;
    k.dtRatio = this.m_inv_dt0 * b;
    k.warmStarting = d.m_warmStarting;
    this.m_contactManager.Collide();
    0 < k.dt && this.Solve(k);
    d.m_continuousPhysics && 0 < k.dt && this.SolveTOI(k);
    0 < k.dt && (this.m_inv_dt0 = k.inv_dt);
    this.m_flags &= ~d.e_locked;
  };
  d.prototype.ClearForces = function () {
    for (var b = this.m_bodyList; b; b = b.m_next)
      b.m_force.SetZero(), (b.m_torque = 0);
  };
  d.prototype.DrawDebugData = function () {
    if (null != this.m_debugDraw) {
      this.m_debugDraw.m_sprite.graphics.clear();
      var b = this.m_debugDraw.GetFlags(),
        a,
        f;
      new c();
      new c();
      new c();
      new A();
      new A();
      new c();
      new c();
      new c();
      new c();
      var r = new h(0, 0, 0);
      if (b & D.e_shapeBit)
        for (a = this.m_bodyList; a; a = a.m_next) {
          var v = a.m_xf;
          for (f = a.GetFixtureList(); f; f = f.m_next) {
            var e = f.GetShape();
            0 == a.IsActive()
              ? r.Set(0.5, 0.5, 0.3)
              : a.GetType() == g.b2_staticBody
              ? r.Set(0.5, 0.9, 0.5)
              : a.GetType() == g.b2_kinematicBody
              ? r.Set(0.5, 0.5, 0.9)
              : 0 == a.IsAwake()
              ? r.Set(0.6, 0.6, 0.6)
              : r.Set(0.9, 0.7, 0.7);
            this.DrawShape(e, v, r);
          }
        }
      if (b & D.e_jointBit)
        for (a = this.m_jointList; a; a = a.m_next) this.DrawJoint(a);
      if (b & D.e_controllerBit)
        for (a = this.m_controllerList; a; a = a.m_next)
          a.Draw(this.m_debugDraw);
      if (b & D.e_pairBit)
        for (
          r.Set(0.3, 0.9, 0.9), a = this.m_contactManager.m_contactList;
          a;
          a = a.GetNext()
        )
          (e = a.GetFixtureA()),
            (f = a.GetFixtureB()),
            (e = e.GetAABB().GetCenter()),
            (f = f.GetAABB().GetCenter()),
            this.m_debugDraw.DrawSegment(e, f, r);
      if (b & D.e_aabbBit)
        for (
          e = this.m_contactManager.m_broadPhase,
            v = [new c(), new c(), new c(), new c()],
            a = this.m_bodyList;
          a;
          a = a.GetNext()
        )
          if (0 != a.IsActive())
            for (f = a.GetFixtureList(); f; f = f.GetNext()) {
              var z = e.GetFatAABB(f.m_proxy);
              v[0].Set(z.lowerBound.x, z.lowerBound.y);
              v[1].Set(z.upperBound.x, z.lowerBound.y);
              v[2].Set(z.upperBound.x, z.upperBound.y);
              v[3].Set(z.lowerBound.x, z.upperBound.y);
              this.m_debugDraw.DrawPolygon(v, 4, r);
            }
      if (b & D.e_centerOfMassBit)
        for (a = this.m_bodyList; a; a = a.m_next)
          (v = d.s_xf),
            (v.R = a.m_xf.R),
            (v.position = a.GetWorldCenter()),
            this.m_debugDraw.DrawTransform(v);
    }
  };
  d.prototype.QueryAABB = function (b, a) {
    var d = this.m_contactManager.m_broadPhase;
    d.Query(function (a) {
      return b(d.GetUserData(a));
    }, a);
  };
  d.prototype.QueryShape = function (b, a, d) {
    void 0 === d && (d = null);
    null == d && ((d = new t()), d.SetIdentity());
    var f = this.m_contactManager.m_broadPhase,
      k = new A();
    a.ComputeAABB(k, d);
    f.Query(function (k) {
      k = f.GetUserData(k) instanceof I ? f.GetUserData(k) : null;
      return u.TestOverlap(a, d, k.GetShape(), k.GetBody().GetTransform())
        ? b(k)
        : !0;
    }, k);
  };
  d.prototype.QueryPoint = function (b, a) {
    var d = this.m_contactManager.m_broadPhase,
      f = new A();
    f.lowerBound.Set(a.x - l.b2_linearSlop, a.y - l.b2_linearSlop);
    f.upperBound.Set(a.x + l.b2_linearSlop, a.y + l.b2_linearSlop);
    d.Query(function (f) {
      f = d.GetUserData(f) instanceof I ? d.GetUserData(f) : null;
      return f.TestPoint(a) ? b(f) : !0;
    }, f);
  };
  d.prototype.RayCast = function (b, a, d) {
    var f = this.m_contactManager.m_broadPhase,
      k = new P(),
      r = new G(a, d);
    f.RayCast(function (r, v) {
      var e = f.GetUserData(v),
        e = e instanceof I ? e : null;
      if (e.RayCast(k, r)) {
        var m = k.fraction,
          q = new c((1 - m) * a.x + m * d.x, (1 - m) * a.y + m * d.y);
        return b(e, q, k.normal, m);
      }
      return r.maxFraction;
    }, r);
  };
  d.prototype.RayCastOne = function (b, a) {
    var d;
    this.RayCast(
      function (b, a, f, k) {
        void 0 === k && (k = 0);
        d = b;
        return k;
      },
      b,
      a
    );
    return d;
  };
  d.prototype.RayCastAll = function (b, a) {
    var d = new Vector();
    this.RayCast(
      function (b, a, f, k) {
        d[d.length] = b;
        return 1;
      },
      b,
      a
    );
    return d;
  };
  d.prototype.GetBodyList = function () {
    return this.m_bodyList;
  };
  d.prototype.GetJointList = function () {
    return this.m_jointList;
  };
  d.prototype.GetContactList = function () {
    return this.m_contactList;
  };
  d.prototype.IsLocked = function () {
    return 0 < (this.m_flags & d.e_locked);
  };
  d.prototype.Solve = function (b) {
    for (var a, d = this.m_controllerList; d; d = d.m_next) d.Step(b);
    d = this.m_island;
    d.Initialize(
      this.m_bodyCount,
      this.m_contactCount,
      this.m_jointCount,
      null,
      this.m_contactManager.m_contactListener,
      this.m_contactSolver
    );
    for (a = this.m_bodyList; a; a = a.m_next) a.m_flags &= ~g.e_islandFlag;
    for (var f = this.m_contactList; f; f = f.m_next)
      f.m_flags &= ~z.e_islandFlag;
    for (f = this.m_jointList; f; f = f.m_next) f.m_islandFlag = !1;
    parseInt(this.m_bodyCount);
    for (var f = this.s_stack, c = this.m_bodyList; c; c = c.m_next)
      if (
        !(c.m_flags & g.e_islandFlag) &&
        0 != c.IsAwake() &&
        0 != c.IsActive() &&
        c.GetType() != g.b2_staticBody
      ) {
        d.Clear();
        var r = 0;
        f[r++] = c;
        for (c.m_flags |= g.e_islandFlag; 0 < r; )
          if (
            ((a = f[--r]),
            d.AddBody(a),
            0 == a.IsAwake() && a.SetAwake(!0),
            a.GetType() != g.b2_staticBody)
          ) {
            for (var v, e = a.m_contactList; e; e = e.next)
              e.contact.m_flags & z.e_islandFlag ||
                1 == e.contact.IsSensor() ||
                0 == e.contact.IsEnabled() ||
                0 == e.contact.IsTouching() ||
                (d.AddContact(e.contact),
                (e.contact.m_flags |= z.e_islandFlag),
                (v = e.other),
                v.m_flags & g.e_islandFlag ||
                  ((f[r++] = v), (v.m_flags |= g.e_islandFlag)));
            for (a = a.m_jointList; a; a = a.next)
              1 != a.joint.m_islandFlag &&
                ((v = a.other),
                0 != v.IsActive() &&
                  (d.AddJoint(a.joint),
                  (a.joint.m_islandFlag = !0),
                  v.m_flags & g.e_islandFlag ||
                    ((f[r++] = v), (v.m_flags |= g.e_islandFlag))));
          }
        d.Solve(b, this.m_gravity, this.m_allowSleep);
        for (r = 0; r < d.m_bodyCount; ++r)
          (a = d.m_bodies[r]),
            a.GetType() == g.b2_staticBody && (a.m_flags &= ~g.e_islandFlag);
      }
    for (r = 0; r < f.length && f[r]; ++r) f[r] = null;
    for (a = this.m_bodyList; a; a = a.m_next)
      0 != a.IsAwake() &&
        0 != a.IsActive() &&
        a.GetType() != g.b2_staticBody &&
        a.SynchronizeFixtures();
    this.m_contactManager.FindNewContacts();
  };
  d.prototype.SolveTOI = function (b) {
    var a,
      f,
      c = this.m_island;
    c.Initialize(
      this.m_bodyCount,
      l.b2_maxTOIContactsPerIsland,
      l.b2_maxTOIJointsPerIsland,
      null,
      this.m_contactManager.m_contactListener,
      this.m_contactSolver
    );
    var r = d.s_queue;
    for (a = this.m_bodyList; a; a = a.m_next)
      (a.m_flags &= ~g.e_islandFlag), (a.m_sweep.t0 = 0);
    for (f = this.m_contactList; f; f = f.m_next)
      f.m_flags &= ~(z.e_toiFlag | z.e_islandFlag);
    for (f = this.m_jointList; f; f = f.m_next) f.m_islandFlag = !1;
    for (;;) {
      var v = null,
        e = 1;
      for (f = this.m_contactList; f; f = f.m_next)
        if (1 != f.IsSensor() && 0 != f.IsEnabled() && 0 != f.IsContinuous()) {
          if (f.m_flags & z.e_toiFlag) var h = f.m_toi;
          else {
            h = f.m_fixtureA;
            a = f.m_fixtureB;
            h = h.m_body;
            a = a.m_body;
            if (
              !(
                (h.GetType() == g.b2_dynamicBody && 0 != h.IsAwake()) ||
                (a.GetType() == g.b2_dynamicBody && 0 != a.IsAwake())
              )
            )
              continue;
            var J = h.m_sweep.t0;
            h.m_sweep.t0 < a.m_sweep.t0
              ? ((J = a.m_sweep.t0), h.m_sweep.Advance(J))
              : a.m_sweep.t0 < h.m_sweep.t0 &&
                ((J = h.m_sweep.t0), a.m_sweep.Advance(J));
            h = f.ComputeTOI(h.m_sweep, a.m_sweep);
            l.b2Assert(0 <= h && 1 >= h);
            0 < h && 1 > h && ((h = (1 - h) * J + h), 1 < h && (h = 1));
            f.m_toi = h;
            f.m_flags |= z.e_toiFlag;
          }
          Number.MIN_VALUE < h && h < e && ((v = f), (e = h));
        }
      if (null == v || 1 - 100 * Number.MIN_VALUE < e) break;
      h = v.m_fixtureA;
      a = v.m_fixtureB;
      h = h.m_body;
      a = a.m_body;
      d.s_backupA.Set(h.m_sweep);
      d.s_backupB.Set(a.m_sweep);
      h.Advance(e);
      a.Advance(e);
      v.Update(this.m_contactManager.m_contactListener);
      v.m_flags &= ~z.e_toiFlag;
      if (1 == v.IsSensor() || 0 == v.IsEnabled())
        h.m_sweep.Set(d.s_backupA),
          a.m_sweep.Set(d.s_backupB),
          h.SynchronizeTransform(),
          a.SynchronizeTransform();
      else if (0 != v.IsTouching()) {
        h.GetType() != g.b2_dynamicBody && (h = a);
        c.Clear();
        v = f = 0;
        r[f + v++] = h;
        for (h.m_flags |= g.e_islandFlag; 0 < v; )
          if (
            ((a = r[f++]),
            --v,
            c.AddBody(a),
            0 == a.IsAwake() && a.SetAwake(!0),
            a.GetType() == g.b2_dynamicBody)
          ) {
            for (
              J = a.m_contactList;
              J && c.m_contactCount != c.m_contactCapacity;
              J = J.next
            )
              J.contact.m_flags & z.e_islandFlag ||
                1 == J.contact.IsSensor() ||
                0 == J.contact.IsEnabled() ||
                0 == J.contact.IsTouching() ||
                (c.AddContact(J.contact),
                (J.contact.m_flags |= z.e_islandFlag),
                (h = J.other),
                h.m_flags & g.e_islandFlag ||
                  (h.GetType() != g.b2_staticBody &&
                    (h.Advance(e), h.SetAwake(!0)),
                  (r[f + v] = h),
                  ++v,
                  (h.m_flags |= g.e_islandFlag)));
            for (a = a.m_jointList; a; a = a.next)
              c.m_jointCount != c.m_jointCapacity &&
                1 != a.joint.m_islandFlag &&
                ((h = a.other),
                0 != h.IsActive() &&
                  (c.AddJoint(a.joint),
                  (a.joint.m_islandFlag = !0),
                  h.m_flags & g.e_islandFlag ||
                    (h.GetType() != g.b2_staticBody &&
                      (h.Advance(e), h.SetAwake(!0)),
                    (r[f + v] = h),
                    ++v,
                    (h.m_flags |= g.e_islandFlag))));
          }
        f = d.s_timestep;
        f.warmStarting = !1;
        f.dt = (1 - e) * b.dt;
        f.inv_dt = 1 / f.dt;
        f.dtRatio = 0;
        f.velocityIterations = b.velocityIterations;
        f.positionIterations = b.positionIterations;
        c.SolveTOI(f);
        for (e = 0; e < c.m_bodyCount; ++e)
          if (
            ((a = c.m_bodies[e]),
            (a.m_flags &= ~g.e_islandFlag),
            0 != a.IsAwake() && a.GetType() == g.b2_dynamicBody)
          )
            for (a.SynchronizeFixtures(), J = a.m_contactList; J; J = J.next)
              J.contact.m_flags &= ~z.e_toiFlag;
        for (e = 0; e < c.m_contactCount; ++e)
          (f = c.m_contacts[e]), (f.m_flags &= ~(z.e_toiFlag | z.e_islandFlag));
        for (e = 0; e < c.m_jointCount; ++e)
          (f = c.m_joints[e]), (f.m_islandFlag = !1);
        this.m_contactManager.FindNewContacts();
      }
    }
  };
  d.prototype.DrawJoint = function (b) {
    var a = b.GetBodyA(),
      f = b.GetBodyB(),
      c = a.m_xf.position,
      r = f.m_xf.position,
      e = b.GetAnchorA(),
      h = b.GetAnchorB(),
      z = d.s_jointColor;
    switch (b.m_type) {
      case v.e_distanceJoint:
        this.m_debugDraw.DrawSegment(e, h, z);
        break;
      case v.e_pulleyJoint:
        a = b instanceof J ? b : null;
        b = a.GetGroundAnchorA();
        a = a.GetGroundAnchorB();
        this.m_debugDraw.DrawSegment(b, e, z);
        this.m_debugDraw.DrawSegment(a, h, z);
        this.m_debugDraw.DrawSegment(b, a, z);
        break;
      case v.e_mouseJoint:
        this.m_debugDraw.DrawSegment(e, h, z);
        break;
      default:
        a != this.m_groundBody && this.m_debugDraw.DrawSegment(c, e, z),
          this.m_debugDraw.DrawSegment(e, h, z),
          f != this.m_groundBody && this.m_debugDraw.DrawSegment(r, h, z);
    }
  };
  d.prototype.DrawShape = function (b, a, f) {
    switch (b.m_type) {
      case u.e_circleShape:
        b = b instanceof H ? b : null;
        var d = n.MulX(a, b.m_p);
        this.m_debugDraw.DrawSolidCircle(d, b.m_radius, a.R.col1, f);
        break;
      case u.e_polygonShape:
        d = b instanceof Q ? b : null;
        b = parseInt(d.GetVertexCount());
        for (var c = d.GetVertices(), r = new Vector(b), d = 0; d < b; ++d)
          r[d] = n.MulX(a, c[d]);
        this.m_debugDraw.DrawSolidPolygon(r, b, f);
        break;
      case u.e_edgeShape:
        (b = b instanceof L ? b : null),
          this.m_debugDraw.DrawSegment(
            n.MulX(a, b.GetVertex1()),
            n.MulX(a, b.GetVertex2()),
            f
          );
    }
  };
  Box2D.postDefs.push(function () {
    Box2D.Dynamics.b2World.s_timestep2 = new a();
    Box2D.Dynamics.b2World.s_xf = new t();
    Box2D.Dynamics.b2World.s_backupA = new x();
    Box2D.Dynamics.b2World.s_backupB = new x();
    Box2D.Dynamics.b2World.s_timestep = new a();
    Box2D.Dynamics.b2World.s_queue = new Vector();
    Box2D.Dynamics.b2World.s_jointColor = new h(0.5, 0.8, 0.8);
    Box2D.Dynamics.b2World.e_newFixture = 1;
    Box2D.Dynamics.b2World.e_locked = 2;
  });
})();
(function () {
  var n = Box2D.Collision.Shapes.b2CircleShape,
    x = Box2D.Collision.Shapes.b2EdgeShape,
    t = Box2D.Collision.Shapes.b2PolygonShape,
    c = Box2D.Collision.Shapes.b2Shape,
    h = Box2D.Dynamics.Contacts.b2CircleContact,
    l = Box2D.Dynamics.Contacts.b2Contact,
    A = Box2D.Dynamics.Contacts.b2ContactConstraint,
    e = Box2D.Dynamics.Contacts.b2ContactConstraintPoint,
    w = Box2D.Dynamics.Contacts.b2ContactEdge,
    G = Box2D.Dynamics.Contacts.b2ContactFactory,
    P = Box2D.Dynamics.Contacts.b2ContactRegister,
    H = Box2D.Dynamics.Contacts.b2ContactResult,
    L = Box2D.Dynamics.Contacts.b2ContactSolver,
    y = Box2D.Dynamics.Contacts.b2EdgeAndCircleContact,
    Q = Box2D.Dynamics.Contacts.b2NullContact,
    u = Box2D.Dynamics.Contacts.b2PolyAndCircleContact,
    g = Box2D.Dynamics.Contacts.b2PolyAndEdgeContact,
    C = Box2D.Dynamics.Contacts.b2PolygonContact,
    E = Box2D.Dynamics.Contacts.b2PositionSolverManifold,
    M = Box2D.Dynamics.b2Body,
    K = Box2D.Dynamics.b2TimeStep,
    N = Box2D.Common.b2Settings,
    D = Box2D.Common.Math.b2Mat22,
    R = Box2D.Common.Math.b2Math,
    F = Box2D.Common.Math.b2Vec2,
    I = Box2D.Collision.b2Collision,
    S = Box2D.Collision.b2ContactID,
    O = Box2D.Collision.b2Manifold,
    a = Box2D.Collision.b2TimeOfImpact,
    d = Box2D.Collision.b2TOIInput,
    z = Box2D.Collision.b2WorldManifold;
  Box2D.inherit(h, Box2D.Dynamics.Contacts.b2Contact);
  h.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
  h.b2CircleContact = function () {
    Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
  };
  h.Create = function (a) {
    return new h();
  };
  h.Destroy = function (a, d) {};
  h.prototype.Reset = function (a, d) {
    this.__super.Reset.call(this, a, d);
  };
  h.prototype.Evaluate = function () {
    var a = this.m_fixtureA.GetBody(),
      d = this.m_fixtureB.GetBody();
    I.CollideCircles(
      this.m_manifold,
      this.m_fixtureA.GetShape() instanceof n
        ? this.m_fixtureA.GetShape()
        : null,
      a.m_xf,
      this.m_fixtureB.GetShape() instanceof n
        ? this.m_fixtureB.GetShape()
        : null,
      d.m_xf
    );
  };
  l.b2Contact = function () {
    this.m_nodeA = new w();
    this.m_nodeB = new w();
    this.m_manifold = new O();
    this.m_oldManifold = new O();
  };
  l.prototype.GetManifold = function () {
    return this.m_manifold;
  };
  l.prototype.GetWorldManifold = function (a) {
    var f = this.m_fixtureA.GetBody(),
      d = this.m_fixtureB.GetBody(),
      c = this.m_fixtureA.GetShape(),
      b = this.m_fixtureB.GetShape();
    a.Initialize(
      this.m_manifold,
      f.GetTransform(),
      c.m_radius,
      d.GetTransform(),
      b.m_radius
    );
  };
  l.prototype.IsTouching = function () {
    return (this.m_flags & l.e_touchingFlag) == l.e_touchingFlag;
  };
  l.prototype.IsContinuous = function () {
    return (this.m_flags & l.e_continuousFlag) == l.e_continuousFlag;
  };
  l.prototype.SetSensor = function (a) {
    this.m_flags = a
      ? this.m_flags | l.e_sensorFlag
      : this.m_flags & ~l.e_sensorFlag;
  };
  l.prototype.IsSensor = function () {
    return (this.m_flags & l.e_sensorFlag) == l.e_sensorFlag;
  };
  l.prototype.SetEnabled = function (a) {
    this.m_flags = a
      ? this.m_flags | l.e_enabledFlag
      : this.m_flags & ~l.e_enabledFlag;
  };
  l.prototype.IsEnabled = function () {
    return (this.m_flags & l.e_enabledFlag) == l.e_enabledFlag;
  };
  l.prototype.GetNext = function () {
    return this.m_next;
  };
  l.prototype.GetFixtureA = function () {
    return this.m_fixtureA;
  };
  l.prototype.GetFixtureB = function () {
    return this.m_fixtureB;
  };
  l.prototype.FlagForFiltering = function () {
    this.m_flags |= l.e_filterFlag;
  };
  l.prototype.b2Contact = function () {};
  l.prototype.Reset = function (a, d) {
    void 0 === a && (a = null);
    void 0 === d && (d = null);
    this.m_flags = l.e_enabledFlag;
    if (a && d) {
      if (a.IsSensor() || d.IsSensor()) this.m_flags |= l.e_sensorFlag;
      var f = a.GetBody(),
        c = d.GetBody();
      if (
        f.GetType() != M.b2_dynamicBody ||
        f.IsBullet() ||
        c.GetType() != M.b2_dynamicBody ||
        c.IsBullet()
      )
        this.m_flags |= l.e_continuousFlag;
      this.m_fixtureA = a;
      this.m_fixtureB = d;
      this.m_manifold.m_pointCount = 0;
      this.m_next = this.m_prev = null;
      this.m_nodeA.contact = null;
      this.m_nodeA.prev = null;
      this.m_nodeA.next = null;
      this.m_nodeA.other = null;
      this.m_nodeB.contact = null;
      this.m_nodeB.prev = null;
      this.m_nodeB.next = null;
      this.m_nodeB.other = null;
    } else this.m_fixtureB = this.m_fixtureA = null;
  };
  l.prototype.Update = function (a) {
    var d = this.m_oldManifold;
    this.m_oldManifold = this.m_manifold;
    this.m_manifold = d;
    this.m_flags |= l.e_enabledFlag;
    var f = !1,
      d = (this.m_flags & l.e_touchingFlag) == l.e_touchingFlag,
      e = this.m_fixtureA.m_body,
      b = this.m_fixtureB.m_body,
      k = this.m_fixtureA.m_aabb.TestOverlap(this.m_fixtureB.m_aabb);
    if (this.m_flags & l.e_sensorFlag)
      k &&
        ((f = this.m_fixtureA.GetShape()),
        (k = this.m_fixtureB.GetShape()),
        (e = e.GetTransform()),
        (b = b.GetTransform()),
        (f = c.TestOverlap(f, e, k, b))),
        (this.m_manifold.m_pointCount = 0);
    else {
      e.GetType() != M.b2_dynamicBody ||
      e.IsBullet() ||
      b.GetType() != M.b2_dynamicBody ||
      b.IsBullet()
        ? (this.m_flags |= l.e_continuousFlag)
        : (this.m_flags &= ~l.e_continuousFlag);
      if (k)
        for (
          this.Evaluate(), f = 0 < this.m_manifold.m_pointCount, k = 0;
          k < this.m_manifold.m_pointCount;
          ++k
        ) {
          var h = this.m_manifold.m_points[k];
          h.m_normalImpulse = 0;
          h.m_tangentImpulse = 0;
          for (
            var z = h.m_id, g = 0;
            g < this.m_oldManifold.m_pointCount;
            ++g
          ) {
            var B = this.m_oldManifold.m_points[g];
            if (B.m_id.key == z.key) {
              h.m_normalImpulse = B.m_normalImpulse;
              h.m_tangentImpulse = B.m_tangentImpulse;
              break;
            }
          }
        }
      else this.m_manifold.m_pointCount = 0;
      f != d && (e.SetAwake(!0), b.SetAwake(!0));
    }
    this.m_flags = f
      ? this.m_flags | l.e_touchingFlag
      : this.m_flags & ~l.e_touchingFlag;
    0 == d && 1 == f && a.BeginContact(this);
    1 == d && 0 == f && a.EndContact(this);
    0 == (this.m_flags & l.e_sensorFlag) &&
      a.PreSolve(this, this.m_oldManifold);
  };
  l.prototype.Evaluate = function () {};
  l.prototype.ComputeTOI = function (d, c) {
    l.s_input.proxyA.Set(this.m_fixtureA.GetShape());
    l.s_input.proxyB.Set(this.m_fixtureB.GetShape());
    l.s_input.sweepA = d;
    l.s_input.sweepB = c;
    l.s_input.tolerance = N.b2_linearSlop;
    return a.TimeOfImpact(l.s_input);
  };
  Box2D.postDefs.push(function () {
    Box2D.Dynamics.Contacts.b2Contact.e_sensorFlag = 1;
    Box2D.Dynamics.Contacts.b2Contact.e_continuousFlag = 2;
    Box2D.Dynamics.Contacts.b2Contact.e_islandFlag = 4;
    Box2D.Dynamics.Contacts.b2Contact.e_toiFlag = 8;
    Box2D.Dynamics.Contacts.b2Contact.e_touchingFlag = 16;
    Box2D.Dynamics.Contacts.b2Contact.e_enabledFlag = 32;
    Box2D.Dynamics.Contacts.b2Contact.e_filterFlag = 64;
    Box2D.Dynamics.Contacts.b2Contact.s_input = new d();
  });
  A.b2ContactConstraint = function () {
    this.localPlaneNormal = new F();
    this.localPoint = new F();
    this.normal = new F();
    this.normalMass = new D();
    this.K = new D();
  };
  A.prototype.b2ContactConstraint = function () {
    this.points = new Vector(N.b2_maxManifoldPoints);
    for (var a = 0; a < N.b2_maxManifoldPoints; a++) this.points[a] = new e();
  };
  e.b2ContactConstraintPoint = function () {
    this.localPoint = new F();
    this.rA = new F();
    this.rB = new F();
  };
  w.b2ContactEdge = function () {};
  G.b2ContactFactory = function () {};
  G.prototype.b2ContactFactory = function (a) {
    this.m_allocator = a;
    this.InitializeRegisters();
  };
  G.prototype.AddType = function (a, d, c, e) {
    void 0 === c && (c = 0);
    void 0 === e && (e = 0);
    this.m_registers[c][e].createFcn = a;
    this.m_registers[c][e].destroyFcn = d;
    this.m_registers[c][e].primary = !0;
    c != e &&
      ((this.m_registers[e][c].createFcn = a),
      (this.m_registers[e][c].destroyFcn = d),
      (this.m_registers[e][c].primary = !1));
  };
  G.prototype.InitializeRegisters = function () {
    this.m_registers = new Vector(c.e_shapeTypeCount);
    for (var a = 0; a < c.e_shapeTypeCount; a++) {
      this.m_registers[a] = new Vector(c.e_shapeTypeCount);
      for (var d = 0; d < c.e_shapeTypeCount; d++)
        this.m_registers[a][d] = new P();
    }
    this.AddType(h.Create, h.Destroy, c.e_circleShape, c.e_circleShape);
    this.AddType(u.Create, u.Destroy, c.e_polygonShape, c.e_circleShape);
    this.AddType(C.Create, C.Destroy, c.e_polygonShape, c.e_polygonShape);
    this.AddType(y.Create, y.Destroy, c.e_edgeShape, c.e_circleShape);
    this.AddType(g.Create, g.Destroy, c.e_polygonShape, c.e_edgeShape);
  };
  G.prototype.Create = function (a, d) {
    var f = parseInt(a.GetType()),
      c = parseInt(d.GetType()),
      f = this.m_registers[f][c];
    if (f.pool)
      return (c = f.pool), (f.pool = c.m_next), f.poolCount--, c.Reset(a, d), c;
    c = f.createFcn;
    return null != c
      ? (f.primary
          ? ((c = c(this.m_allocator)), c.Reset(a, d))
          : ((c = c(this.m_allocator)), c.Reset(d, a)),
        c)
      : null;
  };
  G.prototype.Destroy = function (a) {
    0 < a.m_manifold.m_pointCount &&
      (a.m_fixtureA.m_body.SetAwake(!0), a.m_fixtureB.m_body.SetAwake(!0));
    var d = parseInt(a.m_fixtureA.GetType()),
      f = parseInt(a.m_fixtureB.GetType()),
      d = this.m_registers[d][f];
    d.poolCount++;
    a.m_next = d.pool;
    d.pool = a;
    d = d.destroyFcn;
    d(a, this.m_allocator);
  };
  P.b2ContactRegister = function () {};
  H.b2ContactResult = function () {
    this.position = new F();
    this.normal = new F();
    this.id = new S();
  };
  L.b2ContactSolver = function () {
    this.m_step = new K();
    this.m_constraints = new Vector();
  };
  L.prototype.b2ContactSolver = function () {};
  L.prototype.Initialize = function (a, d, c, e) {
    void 0 === c && (c = 0);
    this.m_step.Set(a);
    this.m_allocator = e;
    for (
      this.m_constraintCount = c;
      this.m_constraints.length < this.m_constraintCount;

    )
      this.m_constraints[this.m_constraints.length] = new A();
    for (a = 0; a < c; ++a) {
      var b = d[a];
      e = b.m_fixtureA;
      var f = b.m_fixtureB,
        r = e.m_shape.m_radius,
        v = f.m_shape.m_radius,
        h = e.m_body,
        z = f.m_body,
        g = b.GetManifold(),
        l = N.b2MixFriction(e.GetFriction(), f.GetFriction()),
        J = N.b2MixRestitution(e.GetRestitution(), f.GetRestitution()),
        n = h.m_linearVelocity.x,
        x = h.m_linearVelocity.y,
        t = z.m_linearVelocity.x,
        u = z.m_linearVelocity.y,
        y = h.m_angularVelocity,
        w = z.m_angularVelocity;
      N.b2Assert(0 < g.m_pointCount);
      L.s_worldManifold.Initialize(g, h.m_xf, r, z.m_xf, v);
      f = L.s_worldManifold.m_normal.x;
      b = L.s_worldManifold.m_normal.y;
      e = this.m_constraints[a];
      e.bodyA = h;
      e.bodyB = z;
      e.manifold = g;
      e.normal.x = f;
      e.normal.y = b;
      e.pointCount = g.m_pointCount;
      e.friction = l;
      e.restitution = J;
      e.localPlaneNormal.x = g.m_localPlaneNormal.x;
      e.localPlaneNormal.y = g.m_localPlaneNormal.y;
      e.localPoint.x = g.m_localPoint.x;
      e.localPoint.y = g.m_localPoint.y;
      e.radius = r + v;
      e.type = g.m_type;
      for (r = 0; r < e.pointCount; ++r) {
        l = g.m_points[r];
        v = e.points[r];
        v.normalImpulse = l.m_normalImpulse;
        v.tangentImpulse = l.m_tangentImpulse;
        v.localPoint.SetV(l.m_localPoint);
        var l = (v.rA.x = L.s_worldManifold.m_points[r].x - h.m_sweep.c.x),
          J = (v.rA.y = L.s_worldManifold.m_points[r].y - h.m_sweep.c.y),
          C = (v.rB.x = L.s_worldManifold.m_points[r].x - z.m_sweep.c.x),
          D = (v.rB.y = L.s_worldManifold.m_points[r].y - z.m_sweep.c.y),
          E = l * b - J * f,
          G = C * b - D * f,
          E = E * E,
          G = G * G;
        v.normalMass =
          1 / (h.m_invMass + z.m_invMass + h.m_invI * E + z.m_invI * G);
        var F = h.m_mass * h.m_invMass + z.m_mass * z.m_invMass,
          F = F + (h.m_mass * h.m_invI * E + z.m_mass * z.m_invI * G);
        v.equalizedMass = 1 / F;
        G = b;
        F = -f;
        E = l * F - J * G;
        G = C * F - D * G;
        E *= E;
        G *= G;
        v.tangentMass =
          1 / (h.m_invMass + z.m_invMass + h.m_invI * E + z.m_invI * G);
        v.velocityBias = 0;
        l =
          e.normal.x * (t + -w * D - n - -y * J) +
          e.normal.y * (u + w * C - x - y * l);
        l < -N.b2_velocityThreshold && (v.velocityBias += -e.restitution * l);
      }
      2 == e.pointCount &&
        ((u = e.points[0]),
        (t = e.points[1]),
        (g = h.m_invMass),
        (h = h.m_invI),
        (n = z.m_invMass),
        (z = z.m_invI),
        (x = u.rA.x * b - u.rA.y * f),
        (u = u.rB.x * b - u.rB.y * f),
        (y = t.rA.x * b - t.rA.y * f),
        (t = t.rB.x * b - t.rB.y * f),
        (f = g + n + h * x * x + z * u * u),
        (b = g + n + h * y * y + z * t * t),
        (z = g + n + h * x * y + z * u * t),
        f * f < 100 * (f * b - z * z)
          ? (e.K.col1.Set(f, z),
            e.K.col2.Set(z, b),
            e.K.GetInverse(e.normalMass))
          : (e.pointCount = 1));
    }
  };
  L.prototype.InitVelocityConstraints = function (a) {
    for (var d = 0; d < this.m_constraintCount; ++d) {
      var f = this.m_constraints[d],
        c = f.bodyA,
        b = f.bodyB,
        k = c.m_invMass,
        e = c.m_invI,
        h = b.m_invMass,
        z = b.m_invI,
        g = f.normal.x,
        l = f.normal.y,
        n = l,
        x = -g,
        t;
      if (a.warmStarting) {
        var u = f.pointCount;
        for (t = 0; t < u; ++t) {
          var y = f.points[t];
          y.normalImpulse *= a.dtRatio;
          y.tangentImpulse *= a.dtRatio;
          var w = y.normalImpulse * g + y.tangentImpulse * n,
            A = y.normalImpulse * l + y.tangentImpulse * x;
          c.m_angularVelocity -= e * (y.rA.x * A - y.rA.y * w);
          c.m_linearVelocity.x -= k * w;
          c.m_linearVelocity.y -= k * A;
          b.m_angularVelocity += z * (y.rB.x * A - y.rB.y * w);
          b.m_linearVelocity.x += h * w;
          b.m_linearVelocity.y += h * A;
        }
      } else
        for (u = f.pointCount, t = 0; t < u; ++t)
          (c = f.points[t]), (c.normalImpulse = 0), (c.tangentImpulse = 0);
    }
  };
  L.prototype.SolveVelocityConstraints = function () {
    for (
      var a, d, c, e, b, k, h, z, g, l, n = 0;
      n < this.m_constraintCount;
      ++n
    ) {
      b = this.m_constraints[n];
      var x = b.bodyA,
        t = b.bodyB,
        u = x.m_angularVelocity,
        y = t.m_angularVelocity,
        w = x.m_linearVelocity,
        A = t.m_linearVelocity,
        C = x.m_invMass,
        D = x.m_invI,
        E = t.m_invMass,
        G = t.m_invI;
      z = b.normal.x;
      var F = (g = b.normal.y);
      l = -z;
      h = b.friction;
      for (a = 0; a < b.pointCount; a++)
        (d = b.points[a]),
          (c = A.x - y * d.rB.y - w.x + u * d.rA.y),
          (e = A.y + y * d.rB.x - w.y - u * d.rA.x),
          (c = c * F + e * l),
          (c = d.tangentMass * -c),
          (e = h * d.normalImpulse),
          (e = R.Clamp(d.tangentImpulse + c, -e, e)),
          (c = e - d.tangentImpulse),
          (k = c * F),
          (c *= l),
          (w.x -= C * k),
          (w.y -= C * c),
          (u -= D * (d.rA.x * c - d.rA.y * k)),
          (A.x += E * k),
          (A.y += E * c),
          (y += G * (d.rB.x * c - d.rB.y * k)),
          (d.tangentImpulse = e);
      parseInt(b.pointCount);
      if (1 == b.pointCount)
        (d = b.points[0]),
          (c = A.x + -y * d.rB.y - w.x - -u * d.rA.y),
          (e = A.y + y * d.rB.x - w.y - u * d.rA.x),
          (b = c * z + e * g),
          (c = -d.normalMass * (b - d.velocityBias)),
          (e = d.normalImpulse + c),
          (e = 0 < e ? e : 0),
          (c = e - d.normalImpulse),
          (k = c * z),
          (c *= g),
          (w.x -= C * k),
          (w.y -= C * c),
          (u -= D * (d.rA.x * c - d.rA.y * k)),
          (A.x += E * k),
          (A.y += E * c),
          (y += G * (d.rB.x * c - d.rB.y * k)),
          (d.normalImpulse = e);
      else {
        d = b.points[0];
        a = b.points[1];
        c = d.normalImpulse;
        h = a.normalImpulse;
        var H =
            (A.x - y * d.rB.y - w.x + u * d.rA.y) * z +
            (A.y + y * d.rB.x - w.y - u * d.rA.x) * g,
          I =
            (A.x - y * a.rB.y - w.x + u * a.rA.y) * z +
            (A.y + y * a.rB.x - w.y - u * a.rA.x) * g;
        e = H - d.velocityBias;
        k = I - a.velocityBias;
        l = b.K;
        e -= l.col1.x * c + l.col2.x * h;
        for (k -= l.col1.y * c + l.col2.y * h; ; ) {
          l = b.normalMass;
          F = -(l.col1.x * e + l.col2.x * k);
          l = -(l.col1.y * e + l.col2.y * k);
          if (0 <= F && 0 <= l) {
            c = F - c;
            h = l - h;
            b = c * z;
            c *= g;
            z *= h;
            g *= h;
            w.x -= C * (b + z);
            w.y -= C * (c + g);
            u -= D * (d.rA.x * c - d.rA.y * b + a.rA.x * g - a.rA.y * z);
            A.x += E * (b + z);
            A.y += E * (c + g);
            y += G * (d.rB.x * c - d.rB.y * b + a.rB.x * g - a.rB.y * z);
            d.normalImpulse = F;
            a.normalImpulse = l;
            break;
          }
          F = -d.normalMass * e;
          l = 0;
          I = b.K.col1.y * F + k;
          if (0 <= F && 0 <= I) {
            c = F - c;
            h = l - h;
            b = c * z;
            c *= g;
            z *= h;
            g *= h;
            w.x -= C * (b + z);
            w.y -= C * (c + g);
            u -= D * (d.rA.x * c - d.rA.y * b + a.rA.x * g - a.rA.y * z);
            A.x += E * (b + z);
            A.y += E * (c + g);
            y += G * (d.rB.x * c - d.rB.y * b + a.rB.x * g - a.rB.y * z);
            d.normalImpulse = F;
            a.normalImpulse = l;
            break;
          }
          F = 0;
          l = -a.normalMass * k;
          H = b.K.col2.x * l + e;
          if (0 <= l && 0 <= H) {
            c = F - c;
            h = l - h;
            b = c * z;
            c *= g;
            z *= h;
            g *= h;
            w.x -= C * (b + z);
            w.y -= C * (c + g);
            u -= D * (d.rA.x * c - d.rA.y * b + a.rA.x * g - a.rA.y * z);
            A.x += E * (b + z);
            A.y += E * (c + g);
            y += G * (d.rB.x * c - d.rB.y * b + a.rB.x * g - a.rB.y * z);
            d.normalImpulse = F;
            a.normalImpulse = l;
            break;
          }
          l = F = 0;
          H = e;
          I = k;
          if (0 <= H && 0 <= I) {
            c = F - c;
            h = l - h;
            b = c * z;
            c *= g;
            z *= h;
            g *= h;
            w.x -= C * (b + z);
            w.y -= C * (c + g);
            u -= D * (d.rA.x * c - d.rA.y * b + a.rA.x * g - a.rA.y * z);
            A.x += E * (b + z);
            A.y += E * (c + g);
            y += G * (d.rB.x * c - d.rB.y * b + a.rB.x * g - a.rB.y * z);
            d.normalImpulse = F;
            a.normalImpulse = l;
            break;
          }
          break;
        }
      }
      x.m_angularVelocity = u;
      t.m_angularVelocity = y;
    }
  };
  L.prototype.FinalizeVelocityConstraints = function () {
    for (var a = 0; a < this.m_constraintCount; ++a)
      for (
        var d = this.m_constraints[a], c = d.manifold, e = 0;
        e < d.pointCount;
        ++e
      ) {
        var b = c.m_points[e],
          k = d.points[e];
        b.m_normalImpulse = k.normalImpulse;
        b.m_tangentImpulse = k.tangentImpulse;
      }
  };
  L.prototype.SolvePositionConstraints = function (a) {
    void 0 === a && (a = 0);
    for (var d = 0, f = 0; f < this.m_constraintCount; f++) {
      var c = this.m_constraints[f],
        b = c.bodyA,
        k = c.bodyB,
        e = b.m_mass * b.m_invMass,
        h = b.m_mass * b.m_invI,
        z = k.m_mass * k.m_invMass,
        g = k.m_mass * k.m_invI;
      L.s_psm.Initialize(c);
      for (var l = L.s_psm.m_normal, n = 0; n < c.pointCount; n++) {
        var x = c.points[n],
          t = L.s_psm.m_points[n],
          u = L.s_psm.m_separations[n],
          w = t.x - b.m_sweep.c.x,
          y = t.y - b.m_sweep.c.y,
          A = t.x - k.m_sweep.c.x,
          t = t.y - k.m_sweep.c.y,
          d = d < u ? d : u,
          u = R.Clamp(a * (u + N.b2_linearSlop), -N.b2_maxLinearCorrection, 0),
          u = -x.equalizedMass * u,
          x = u * l.x,
          u = u * l.y;
        b.m_sweep.c.x -= e * x;
        b.m_sweep.c.y -= e * u;
        b.m_sweep.a -= h * (w * u - y * x);
        b.SynchronizeTransform();
        k.m_sweep.c.x += z * x;
        k.m_sweep.c.y += z * u;
        k.m_sweep.a += g * (A * u - t * x);
        k.SynchronizeTransform();
      }
    }
    return d > -1.5 * N.b2_linearSlop;
  };
  Box2D.postDefs.push(function () {
    Box2D.Dynamics.Contacts.b2ContactSolver.s_worldManifold = new z();
    Box2D.Dynamics.Contacts.b2ContactSolver.s_psm = new E();
  });
  Box2D.inherit(y, Box2D.Dynamics.Contacts.b2Contact);
  y.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
  y.b2EdgeAndCircleContact = function () {
    Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
  };
  y.Create = function (a) {
    return new y();
  };
  y.Destroy = function (a, d) {};
  y.prototype.Reset = function (a, d) {
    this.__super.Reset.call(this, a, d);
  };
  y.prototype.Evaluate = function () {
    var a = this.m_fixtureA.GetBody(),
      d = this.m_fixtureB.GetBody();
    this.b2CollideEdgeAndCircle(
      this.m_manifold,
      this.m_fixtureA.GetShape() instanceof x
        ? this.m_fixtureA.GetShape()
        : null,
      a.m_xf,
      this.m_fixtureB.GetShape() instanceof n
        ? this.m_fixtureB.GetShape()
        : null,
      d.m_xf
    );
  };
  y.prototype.b2CollideEdgeAndCircle = function (a, d, c, e, b) {};
  Box2D.inherit(Q, Box2D.Dynamics.Contacts.b2Contact);
  Q.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
  Q.b2NullContact = function () {
    Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
  };
  Q.prototype.b2NullContact = function () {
    this.__super.b2Contact.call(this);
  };
  Q.prototype.Evaluate = function () {};
  Box2D.inherit(u, Box2D.Dynamics.Contacts.b2Contact);
  u.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
  u.b2PolyAndCircleContact = function () {
    Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
  };
  u.Create = function (a) {
    return new u();
  };
  u.Destroy = function (a, d) {};
  u.prototype.Reset = function (a, d) {
    this.__super.Reset.call(this, a, d);
    N.b2Assert(a.GetType() == c.e_polygonShape);
    N.b2Assert(d.GetType() == c.e_circleShape);
  };
  u.prototype.Evaluate = function () {
    var a = this.m_fixtureA.m_body,
      d = this.m_fixtureB.m_body;
    I.CollidePolygonAndCircle(
      this.m_manifold,
      this.m_fixtureA.GetShape() instanceof t
        ? this.m_fixtureA.GetShape()
        : null,
      a.m_xf,
      this.m_fixtureB.GetShape() instanceof n
        ? this.m_fixtureB.GetShape()
        : null,
      d.m_xf
    );
  };
  Box2D.inherit(g, Box2D.Dynamics.Contacts.b2Contact);
  g.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
  g.b2PolyAndEdgeContact = function () {
    Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
  };
  g.Create = function (a) {
    return new g();
  };
  g.Destroy = function (a, d) {};
  g.prototype.Reset = function (a, d) {
    this.__super.Reset.call(this, a, d);
    N.b2Assert(a.GetType() == c.e_polygonShape);
    N.b2Assert(d.GetType() == c.e_edgeShape);
  };
  g.prototype.Evaluate = function () {
    var a = this.m_fixtureA.GetBody(),
      d = this.m_fixtureB.GetBody();
    this.b2CollidePolyAndEdge(
      this.m_manifold,
      this.m_fixtureA.GetShape() instanceof t
        ? this.m_fixtureA.GetShape()
        : null,
      a.m_xf,
      this.m_fixtureB.GetShape() instanceof x
        ? this.m_fixtureB.GetShape()
        : null,
      d.m_xf
    );
  };
  g.prototype.b2CollidePolyAndEdge = function (a, d, c, e, b) {};
  Box2D.inherit(C, Box2D.Dynamics.Contacts.b2Contact);
  C.prototype.__super = Box2D.Dynamics.Contacts.b2Contact.prototype;
  C.b2PolygonContact = function () {
    Box2D.Dynamics.Contacts.b2Contact.b2Contact.apply(this, arguments);
  };
  C.Create = function (a) {
    return new C();
  };
  C.Destroy = function (a, d) {};
  C.prototype.Reset = function (a, d) {
    this.__super.Reset.call(this, a, d);
  };
  C.prototype.Evaluate = function () {
    var a = this.m_fixtureA.GetBody(),
      d = this.m_fixtureB.GetBody();
    I.CollidePolygons(
      this.m_manifold,
      this.m_fixtureA.GetShape() instanceof t
        ? this.m_fixtureA.GetShape()
        : null,
      a.m_xf,
      this.m_fixtureB.GetShape() instanceof t
        ? this.m_fixtureB.GetShape()
        : null,
      d.m_xf
    );
  };
  E.b2PositionSolverManifold = function () {};
  E.prototype.b2PositionSolverManifold = function () {
    this.m_normal = new F();
    this.m_separations = new Vector_a2j_Number(N.b2_maxManifoldPoints);
    this.m_points = new Vector(N.b2_maxManifoldPoints);
    for (var a = 0; a < N.b2_maxManifoldPoints; a++) this.m_points[a] = new F();
  };
  E.prototype.Initialize = function (a) {
    N.b2Assert(0 < a.pointCount);
    switch (a.type) {
      case O.e_circles:
        var d = a.bodyA.m_xf.R;
        var c = a.localPoint;
        var f = a.bodyA.m_xf.position.x + (d.col1.x * c.x + d.col2.x * c.y);
        var b = a.bodyA.m_xf.position.y + (d.col1.y * c.x + d.col2.y * c.y);
        d = a.bodyB.m_xf.R;
        c = a.points[0].localPoint;
        var k = a.bodyB.m_xf.position.x + (d.col1.x * c.x + d.col2.x * c.y);
        d = a.bodyB.m_xf.position.y + (d.col1.y * c.x + d.col2.y * c.y);
        c = k - f;
        var e = d - b;
        var h = c * c + e * e;
        h > Number.MIN_VALUE * Number.MIN_VALUE
          ? ((h = Math.sqrt(h)),
            (this.m_normal.x = c / h),
            (this.m_normal.y = e / h))
          : ((this.m_normal.x = 1), (this.m_normal.y = 0));
        this.m_points[0].x = 0.5 * (f + k);
        this.m_points[0].y = 0.5 * (b + d);
        this.m_separations[0] =
          c * this.m_normal.x + e * this.m_normal.y - a.radius;
        break;
      case O.e_faceA:
        d = a.bodyA.m_xf.R;
        c = a.localPlaneNormal;
        this.m_normal.x = d.col1.x * c.x + d.col2.x * c.y;
        this.m_normal.y = d.col1.y * c.x + d.col2.y * c.y;
        d = a.bodyA.m_xf.R;
        c = a.localPoint;
        k = a.bodyA.m_xf.position.x + (d.col1.x * c.x + d.col2.x * c.y);
        e = a.bodyA.m_xf.position.y + (d.col1.y * c.x + d.col2.y * c.y);
        d = a.bodyB.m_xf.R;
        for (f = 0; f < a.pointCount; ++f)
          (c = a.points[f].localPoint),
            (b = a.bodyB.m_xf.position.x + (d.col1.x * c.x + d.col2.x * c.y)),
            (c = a.bodyB.m_xf.position.y + (d.col1.y * c.x + d.col2.y * c.y)),
            (this.m_separations[f] =
              (b - k) * this.m_normal.x + (c - e) * this.m_normal.y - a.radius),
            (this.m_points[f].x = b),
            (this.m_points[f].y = c);
        break;
      case O.e_faceB:
        d = a.bodyB.m_xf.R;
        c = a.localPlaneNormal;
        this.m_normal.x = d.col1.x * c.x + d.col2.x * c.y;
        this.m_normal.y = d.col1.y * c.x + d.col2.y * c.y;
        d = a.bodyB.m_xf.R;
        c = a.localPoint;
        k = a.bodyB.m_xf.position.x + (d.col1.x * c.x + d.col2.x * c.y);
        e = a.bodyB.m_xf.position.y + (d.col1.y * c.x + d.col2.y * c.y);
        d = a.bodyA.m_xf.R;
        for (f = 0; f < a.pointCount; ++f)
          (c = a.points[f].localPoint),
            (b = a.bodyA.m_xf.position.x + (d.col1.x * c.x + d.col2.x * c.y)),
            (c = a.bodyA.m_xf.position.y + (d.col1.y * c.x + d.col2.y * c.y)),
            (this.m_separations[f] =
              (b - k) * this.m_normal.x + (c - e) * this.m_normal.y - a.radius),
            this.m_points[f].Set(b, c);
        this.m_normal.x *= -1;
        this.m_normal.y *= -1;
    }
  };
  Box2D.postDefs.push(function () {
    Box2D.Dynamics.Contacts.b2PositionSolverManifold.circlePointA = new F();
    Box2D.Dynamics.Contacts.b2PositionSolverManifold.circlePointB = new F();
  });
})();
(function () {
  var n = Box2D.Common.b2Settings,
    x = Box2D.Common.Math.b2Mat22,
    t = Box2D.Common.Math.b2Mat33,
    c = Box2D.Common.Math.b2Math,
    h = Box2D.Common.Math.b2Vec2,
    l = Box2D.Common.Math.b2Vec3,
    A = Box2D.Dynamics.Joints.b2DistanceJoint,
    e = Box2D.Dynamics.Joints.b2DistanceJointDef,
    w = Box2D.Dynamics.Joints.b2FrictionJoint,
    G = Box2D.Dynamics.Joints.b2FrictionJointDef,
    P = Box2D.Dynamics.Joints.b2GearJoint,
    H = Box2D.Dynamics.Joints.b2GearJointDef,
    L = Box2D.Dynamics.Joints.b2Jacobian,
    y = Box2D.Dynamics.Joints.b2Joint,
    Q = Box2D.Dynamics.Joints.b2JointDef,
    u = Box2D.Dynamics.Joints.b2JointEdge,
    g = Box2D.Dynamics.Joints.b2LineJoint,
    C = Box2D.Dynamics.Joints.b2LineJointDef,
    E = Box2D.Dynamics.Joints.b2MouseJoint,
    M = Box2D.Dynamics.Joints.b2MouseJointDef,
    K = Box2D.Dynamics.Joints.b2PrismaticJoint,
    N = Box2D.Dynamics.Joints.b2PrismaticJointDef,
    D = Box2D.Dynamics.Joints.b2PulleyJoint,
    R = Box2D.Dynamics.Joints.b2PulleyJointDef,
    F = Box2D.Dynamics.Joints.b2RevoluteJoint,
    I = Box2D.Dynamics.Joints.b2RevoluteJointDef,
    S = Box2D.Dynamics.Joints.b2WeldJoint,
    O = Box2D.Dynamics.Joints.b2WeldJointDef;
  Box2D.inherit(A, Box2D.Dynamics.Joints.b2Joint);
  A.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
  A.b2DistanceJoint = function () {
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
    this.m_localAnchor1 = new h();
    this.m_localAnchor2 = new h();
    this.m_u = new h();
  };
  A.prototype.GetAnchorA = function () {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
  };
  A.prototype.GetAnchorB = function () {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
  };
  A.prototype.GetReactionForce = function (a) {
    void 0 === a && (a = 0);
    return new h(
      a * this.m_impulse * this.m_u.x,
      a * this.m_impulse * this.m_u.y
    );
  };
  A.prototype.GetReactionTorque = function (a) {
    return 0;
  };
  A.prototype.GetLength = function () {
    return this.m_length;
  };
  A.prototype.SetLength = function (a) {
    void 0 === a && (a = 0);
    this.m_length = a;
  };
  A.prototype.GetFrequency = function () {
    return this.m_frequencyHz;
  };
  A.prototype.SetFrequency = function (a) {
    void 0 === a && (a = 0);
    this.m_frequencyHz = a;
  };
  A.prototype.GetDampingRatio = function () {
    return this.m_dampingRatio;
  };
  A.prototype.SetDampingRatio = function (a) {
    void 0 === a && (a = 0);
    this.m_dampingRatio = a;
  };
  A.prototype.b2DistanceJoint = function (a) {
    this.__super.b2Joint.call(this, a);
    this.m_localAnchor1.SetV(a.localAnchorA);
    this.m_localAnchor2.SetV(a.localAnchorB);
    this.m_length = a.length;
    this.m_frequencyHz = a.frequencyHz;
    this.m_dampingRatio = a.dampingRatio;
    this.m_bias = this.m_gamma = this.m_impulse = 0;
  };
  A.prototype.InitVelocityConstraints = function (a) {
    var d = this.m_bodyA,
      c = this.m_bodyB;
    var f = d.m_xf.R;
    var e = this.m_localAnchor1.x - d.m_sweep.localCenter.x,
      h = this.m_localAnchor1.y - d.m_sweep.localCenter.y;
    var g = f.col1.x * e + f.col2.x * h;
    h = f.col1.y * e + f.col2.y * h;
    e = g;
    f = c.m_xf.R;
    var b = this.m_localAnchor2.x - c.m_sweep.localCenter.x,
      k = this.m_localAnchor2.y - c.m_sweep.localCenter.y;
    g = f.col1.x * b + f.col2.x * k;
    k = f.col1.y * b + f.col2.y * k;
    b = g;
    this.m_u.x = c.m_sweep.c.x + b - d.m_sweep.c.x - e;
    this.m_u.y = c.m_sweep.c.y + k - d.m_sweep.c.y - h;
    g = Math.sqrt(this.m_u.x * this.m_u.x + this.m_u.y * this.m_u.y);
    g > n.b2_linearSlop ? this.m_u.Multiply(1 / g) : this.m_u.SetZero();
    f = e * this.m_u.y - h * this.m_u.x;
    var l = b * this.m_u.y - k * this.m_u.x;
    f = d.m_invMass + d.m_invI * f * f + c.m_invMass + c.m_invI * l * l;
    this.m_mass = 0 != f ? 1 / f : 0;
    if (0 < this.m_frequencyHz) {
      g -= this.m_length;
      var l = 2 * Math.PI * this.m_frequencyHz,
        m = this.m_mass * l * l;
      this.m_gamma =
        a.dt * (2 * this.m_mass * this.m_dampingRatio * l + a.dt * m);
      this.m_gamma = 0 != this.m_gamma ? 1 / this.m_gamma : 0;
      this.m_bias = g * a.dt * m * this.m_gamma;
      this.m_mass = f + this.m_gamma;
      this.m_mass = 0 != this.m_mass ? 1 / this.m_mass : 0;
    }
    a.warmStarting
      ? ((this.m_impulse *= a.dtRatio),
        (a = this.m_impulse * this.m_u.x),
        (f = this.m_impulse * this.m_u.y),
        (d.m_linearVelocity.x -= d.m_invMass * a),
        (d.m_linearVelocity.y -= d.m_invMass * f),
        (d.m_angularVelocity -= d.m_invI * (e * f - h * a)),
        (c.m_linearVelocity.x += c.m_invMass * a),
        (c.m_linearVelocity.y += c.m_invMass * f),
        (c.m_angularVelocity += c.m_invI * (b * f - k * a)))
      : (this.m_impulse = 0);
  };
  A.prototype.SolveVelocityConstraints = function (a) {
    a = this.m_bodyA;
    var d = this.m_bodyB;
    var c = a.m_xf.R;
    var f = this.m_localAnchor1.x - a.m_sweep.localCenter.x,
      e = this.m_localAnchor1.y - a.m_sweep.localCenter.y,
      h = c.col1.x * f + c.col2.x * e,
      e = c.col1.y * f + c.col2.y * e,
      f = h;
    c = d.m_xf.R;
    var g = this.m_localAnchor2.x - d.m_sweep.localCenter.x,
      b = this.m_localAnchor2.y - d.m_sweep.localCenter.y,
      h = c.col1.x * g + c.col2.x * b,
      b = c.col1.y * g + c.col2.y * b,
      g = h,
      h =
        -this.m_mass *
        (this.m_u.x *
          (d.m_linearVelocity.x +
            -d.m_angularVelocity * b -
            (a.m_linearVelocity.x + -a.m_angularVelocity * e)) +
          this.m_u.y *
            (d.m_linearVelocity.y +
              d.m_angularVelocity * g -
              (a.m_linearVelocity.y + a.m_angularVelocity * f)) +
          this.m_bias +
          this.m_gamma * this.m_impulse);
    this.m_impulse += h;
    c = h * this.m_u.x;
    h *= this.m_u.y;
    a.m_linearVelocity.x -= a.m_invMass * c;
    a.m_linearVelocity.y -= a.m_invMass * h;
    a.m_angularVelocity -= a.m_invI * (f * h - e * c);
    d.m_linearVelocity.x += d.m_invMass * c;
    d.m_linearVelocity.y += d.m_invMass * h;
    d.m_angularVelocity += d.m_invI * (g * h - b * c);
  };
  A.prototype.SolvePositionConstraints = function (a) {
    if (0 < this.m_frequencyHz) return !0;
    a = this.m_bodyA;
    var d = this.m_bodyB;
    var e = a.m_xf.R;
    var f = this.m_localAnchor1.x - a.m_sweep.localCenter.x,
      h = this.m_localAnchor1.y - a.m_sweep.localCenter.y,
      g = e.col1.x * f + e.col2.x * h,
      h = e.col1.y * f + e.col2.y * h,
      f = g;
    e = d.m_xf.R;
    var l = this.m_localAnchor2.x - d.m_sweep.localCenter.x,
      b = this.m_localAnchor2.y - d.m_sweep.localCenter.y,
      g = e.col1.x * l + e.col2.x * b,
      b = e.col1.y * l + e.col2.y * b,
      l = g,
      g = d.m_sweep.c.x + l - a.m_sweep.c.x - f,
      k = d.m_sweep.c.y + b - a.m_sweep.c.y - h;
    e = Math.sqrt(g * g + k * k);
    g /= e;
    k /= e;
    e -= this.m_length;
    e = c.Clamp(e, -n.b2_maxLinearCorrection, n.b2_maxLinearCorrection);
    var q = -this.m_mass * e;
    this.m_u.Set(g, k);
    g = q * this.m_u.x;
    k = q * this.m_u.y;
    a.m_sweep.c.x -= a.m_invMass * g;
    a.m_sweep.c.y -= a.m_invMass * k;
    a.m_sweep.a -= a.m_invI * (f * k - h * g);
    d.m_sweep.c.x += d.m_invMass * g;
    d.m_sweep.c.y += d.m_invMass * k;
    d.m_sweep.a += d.m_invI * (l * k - b * g);
    a.SynchronizeTransform();
    d.SynchronizeTransform();
    return c.Abs(e) < n.b2_linearSlop;
  };
  Box2D.inherit(e, Box2D.Dynamics.Joints.b2JointDef);
  e.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
  e.b2DistanceJointDef = function () {
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
    this.localAnchorA = new h();
    this.localAnchorB = new h();
  };
  e.prototype.b2DistanceJointDef = function () {
    this.__super.b2JointDef.call(this);
    this.type = y.e_distanceJoint;
    this.length = 1;
    this.dampingRatio = this.frequencyHz = 0;
  };
  e.prototype.Initialize = function (a, d, c, f) {
    this.bodyA = a;
    this.bodyB = d;
    this.localAnchorA.SetV(this.bodyA.GetLocalPoint(c));
    this.localAnchorB.SetV(this.bodyB.GetLocalPoint(f));
    a = f.x - c.x;
    c = f.y - c.y;
    this.length = Math.sqrt(a * a + c * c);
    this.dampingRatio = this.frequencyHz = 0;
  };
  Box2D.inherit(w, Box2D.Dynamics.Joints.b2Joint);
  w.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
  w.b2FrictionJoint = function () {
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
    this.m_localAnchorA = new h();
    this.m_localAnchorB = new h();
    this.m_linearMass = new x();
    this.m_linearImpulse = new h();
  };
  w.prototype.GetAnchorA = function () {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
  };
  w.prototype.GetAnchorB = function () {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
  };
  w.prototype.GetReactionForce = function (a) {
    void 0 === a && (a = 0);
    return new h(a * this.m_linearImpulse.x, a * this.m_linearImpulse.y);
  };
  w.prototype.GetReactionTorque = function (a) {
    void 0 === a && (a = 0);
    return a * this.m_angularImpulse;
  };
  w.prototype.SetMaxForce = function (a) {
    void 0 === a && (a = 0);
    this.m_maxForce = a;
  };
  w.prototype.GetMaxForce = function () {
    return this.m_maxForce;
  };
  w.prototype.SetMaxTorque = function (a) {
    void 0 === a && (a = 0);
    this.m_maxTorque = a;
  };
  w.prototype.GetMaxTorque = function () {
    return this.m_maxTorque;
  };
  w.prototype.b2FrictionJoint = function (a) {
    this.__super.b2Joint.call(this, a);
    this.m_localAnchorA.SetV(a.localAnchorA);
    this.m_localAnchorB.SetV(a.localAnchorB);
    this.m_linearMass.SetZero();
    this.m_angularMass = 0;
    this.m_linearImpulse.SetZero();
    this.m_angularImpulse = 0;
    this.m_maxForce = a.maxForce;
    this.m_maxTorque = a.maxTorque;
  };
  w.prototype.InitVelocityConstraints = function (a) {
    var d = this.m_bodyA,
      c = this.m_bodyB;
    var f = d.m_xf.R;
    var e = this.m_localAnchorA.x - d.m_sweep.localCenter.x,
      h = this.m_localAnchorA.y - d.m_sweep.localCenter.y;
    var g = f.col1.x * e + f.col2.x * h;
    h = f.col1.y * e + f.col2.y * h;
    e = g;
    f = c.m_xf.R;
    var b = this.m_localAnchorB.x - c.m_sweep.localCenter.x,
      k = this.m_localAnchorB.y - c.m_sweep.localCenter.y;
    g = f.col1.x * b + f.col2.x * k;
    k = f.col1.y * b + f.col2.y * k;
    b = g;
    f = d.m_invMass;
    g = c.m_invMass;
    var l = d.m_invI,
      m = c.m_invI,
      p = new x();
    p.col1.x = f + g;
    p.col2.x = 0;
    p.col1.y = 0;
    p.col2.y = f + g;
    p.col1.x += l * h * h;
    p.col2.x += -l * e * h;
    p.col1.y += -l * e * h;
    p.col2.y += l * e * e;
    p.col1.x += m * k * k;
    p.col2.x += -m * b * k;
    p.col1.y += -m * b * k;
    p.col2.y += m * b * b;
    p.GetInverse(this.m_linearMass);
    this.m_angularMass = l + m;
    0 < this.m_angularMass && (this.m_angularMass = 1 / this.m_angularMass);
    a.warmStarting
      ? ((this.m_linearImpulse.x *= a.dtRatio),
        (this.m_linearImpulse.y *= a.dtRatio),
        (this.m_angularImpulse *= a.dtRatio),
        (a = this.m_linearImpulse),
        (d.m_linearVelocity.x -= f * a.x),
        (d.m_linearVelocity.y -= f * a.y),
        (d.m_angularVelocity -=
          l * (e * a.y - h * a.x + this.m_angularImpulse)),
        (c.m_linearVelocity.x += g * a.x),
        (c.m_linearVelocity.y += g * a.y),
        (c.m_angularVelocity +=
          m * (b * a.y - k * a.x + this.m_angularImpulse)))
      : (this.m_linearImpulse.SetZero(), (this.m_angularImpulse = 0));
  };
  w.prototype.SolveVelocityConstraints = function (a) {
    var d = this.m_bodyA,
      e = this.m_bodyB,
      f = d.m_linearVelocity,
      g = d.m_angularVelocity,
      v = e.m_linearVelocity,
      l = e.m_angularVelocity,
      b = d.m_invMass,
      k = e.m_invMass,
      q = d.m_invI,
      m = e.m_invI;
    var p = d.m_xf.R;
    var n = this.m_localAnchorA.x - d.m_sweep.localCenter.x,
      x = this.m_localAnchorA.y - d.m_sweep.localCenter.y;
    var t = p.col1.x * n + p.col2.x * x;
    x = p.col1.y * n + p.col2.y * x;
    n = t;
    p = e.m_xf.R;
    var u = this.m_localAnchorB.x - e.m_sweep.localCenter.x,
      w = this.m_localAnchorB.y - e.m_sweep.localCenter.y;
    t = p.col1.x * u + p.col2.x * w;
    w = p.col1.y * u + p.col2.y * w;
    u = t;
    t = -this.m_angularMass * (l - g);
    var y = this.m_angularImpulse;
    p = a.dt * this.m_maxTorque;
    this.m_angularImpulse = c.Clamp(this.m_angularImpulse + t, -p, p);
    t = this.m_angularImpulse - y;
    g -= q * t;
    l += m * t;
    p = c.MulMV(
      this.m_linearMass,
      new h(-(v.x - l * w - f.x + g * x), -(v.y + l * u - f.y - g * n))
    );
    t = this.m_linearImpulse.Copy();
    this.m_linearImpulse.Add(p);
    p = a.dt * this.m_maxForce;
    this.m_linearImpulse.LengthSquared() > p * p &&
      (this.m_linearImpulse.Normalize(), this.m_linearImpulse.Multiply(p));
    p = c.SubtractVV(this.m_linearImpulse, t);
    f.x -= b * p.x;
    f.y -= b * p.y;
    g -= q * (n * p.y - x * p.x);
    v.x += k * p.x;
    v.y += k * p.y;
    l += m * (u * p.y - w * p.x);
    d.m_angularVelocity = g;
    e.m_angularVelocity = l;
  };
  w.prototype.SolvePositionConstraints = function (a) {
    return !0;
  };
  Box2D.inherit(G, Box2D.Dynamics.Joints.b2JointDef);
  G.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
  G.b2FrictionJointDef = function () {
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
    this.localAnchorA = new h();
    this.localAnchorB = new h();
  };
  G.prototype.b2FrictionJointDef = function () {
    this.__super.b2JointDef.call(this);
    this.type = y.e_frictionJoint;
    this.maxTorque = this.maxForce = 0;
  };
  G.prototype.Initialize = function (a, d, c) {
    this.bodyA = a;
    this.bodyB = d;
    this.localAnchorA.SetV(this.bodyA.GetLocalPoint(c));
    this.localAnchorB.SetV(this.bodyB.GetLocalPoint(c));
  };
  Box2D.inherit(P, Box2D.Dynamics.Joints.b2Joint);
  P.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
  P.b2GearJoint = function () {
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
    this.m_groundAnchor1 = new h();
    this.m_groundAnchor2 = new h();
    this.m_localAnchor1 = new h();
    this.m_localAnchor2 = new h();
    this.m_J = new L();
  };
  P.prototype.GetAnchorA = function () {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
  };
  P.prototype.GetAnchorB = function () {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
  };
  P.prototype.GetReactionForce = function (a) {
    void 0 === a && (a = 0);
    return new h(
      a * this.m_impulse * this.m_J.linearB.x,
      a * this.m_impulse * this.m_J.linearB.y
    );
  };
  P.prototype.GetReactionTorque = function (a) {
    void 0 === a && (a = 0);
    var d = this.m_bodyB.m_xf.R,
      c = this.m_localAnchor1.x - this.m_bodyB.m_sweep.localCenter.x,
      f = this.m_localAnchor1.y - this.m_bodyB.m_sweep.localCenter.y,
      e = d.col1.x * c + d.col2.x * f,
      f = d.col1.y * c + d.col2.y * f;
    return (
      a *
      (this.m_impulse * this.m_J.angularB -
        e * this.m_impulse * this.m_J.linearB.y +
        f * this.m_impulse * this.m_J.linearB.x)
    );
  };
  P.prototype.GetRatio = function () {
    return this.m_ratio;
  };
  P.prototype.SetRatio = function (a) {
    void 0 === a && (a = 0);
    this.m_ratio = a;
  };
  P.prototype.b2GearJoint = function (a) {
    this.__super.b2Joint.call(this, a);
    var d = parseInt(a.joint1.m_type),
      c = parseInt(a.joint2.m_type);
    this.m_prismatic2 =
      this.m_revolute2 =
      this.m_prismatic1 =
      this.m_revolute1 =
        null;
    this.m_ground1 = a.joint1.GetBodyA();
    this.m_bodyA = a.joint1.GetBodyB();
    d == y.e_revoluteJoint
      ? ((this.m_revolute1 = a.joint1 instanceof F ? a.joint1 : null),
        this.m_groundAnchor1.SetV(this.m_revolute1.m_localAnchor1),
        this.m_localAnchor1.SetV(this.m_revolute1.m_localAnchor2),
        (d = this.m_revolute1.GetJointAngle()))
      : ((this.m_prismatic1 = a.joint1 instanceof K ? a.joint1 : null),
        this.m_groundAnchor1.SetV(this.m_prismatic1.m_localAnchor1),
        this.m_localAnchor1.SetV(this.m_prismatic1.m_localAnchor2),
        (d = this.m_prismatic1.GetJointTranslation()));
    this.m_ground2 = a.joint2.GetBodyA();
    this.m_bodyB = a.joint2.GetBodyB();
    c == y.e_revoluteJoint
      ? ((this.m_revolute2 = a.joint2 instanceof F ? a.joint2 : null),
        this.m_groundAnchor2.SetV(this.m_revolute2.m_localAnchor1),
        this.m_localAnchor2.SetV(this.m_revolute2.m_localAnchor2),
        (c = this.m_revolute2.GetJointAngle()))
      : ((this.m_prismatic2 = a.joint2 instanceof K ? a.joint2 : null),
        this.m_groundAnchor2.SetV(this.m_prismatic2.m_localAnchor1),
        this.m_localAnchor2.SetV(this.m_prismatic2.m_localAnchor2),
        (c = this.m_prismatic2.GetJointTranslation()));
    this.m_ratio = a.ratio;
    this.m_constant = d + this.m_ratio * c;
    this.m_impulse = 0;
  };
  P.prototype.InitVelocityConstraints = function (a) {
    var d = this.m_ground1,
      c = this.m_ground2,
      f = this.m_bodyA,
      e = this.m_bodyB,
      h = 0;
    this.m_J.SetZero();
    if (this.m_revolute1) (this.m_J.angularA = -1), (h += f.m_invI);
    else {
      var g = d.m_xf.R;
      var b = this.m_prismatic1.m_localXAxis1;
      d = g.col1.x * b.x + g.col2.x * b.y;
      b = g.col1.y * b.x + g.col2.y * b.y;
      g = f.m_xf.R;
      var k = this.m_localAnchor1.x - f.m_sweep.localCenter.x;
      var l = this.m_localAnchor1.y - f.m_sweep.localCenter.y;
      var m = g.col1.x * k + g.col2.x * l;
      l = g.col1.y * k + g.col2.y * l;
      g = m * b - l * d;
      this.m_J.linearA.Set(-d, -b);
      this.m_J.angularA = -g;
      h += f.m_invMass + f.m_invI * g * g;
    }
    this.m_revolute2
      ? ((this.m_J.angularB = -this.m_ratio),
        (h += this.m_ratio * this.m_ratio * e.m_invI))
      : ((g = c.m_xf.R),
        (b = this.m_prismatic2.m_localXAxis1),
        (d = g.col1.x * b.x + g.col2.x * b.y),
        (b = g.col1.y * b.x + g.col2.y * b.y),
        (g = e.m_xf.R),
        (k = this.m_localAnchor2.x - e.m_sweep.localCenter.x),
        (l = this.m_localAnchor2.y - e.m_sweep.localCenter.y),
        (m = g.col1.x * k + g.col2.x * l),
        (l = g.col1.y * k + g.col2.y * l),
        (g = m * b - l * d),
        this.m_J.linearB.Set(-this.m_ratio * d, -this.m_ratio * b),
        (this.m_J.angularB = -this.m_ratio * g),
        (h += this.m_ratio * this.m_ratio * (e.m_invMass + e.m_invI * g * g)));
    this.m_mass = 0 < h ? 1 / h : 0;
    a.warmStarting
      ? ((f.m_linearVelocity.x +=
          f.m_invMass * this.m_impulse * this.m_J.linearA.x),
        (f.m_linearVelocity.y +=
          f.m_invMass * this.m_impulse * this.m_J.linearA.y),
        (f.m_angularVelocity += f.m_invI * this.m_impulse * this.m_J.angularA),
        (e.m_linearVelocity.x +=
          e.m_invMass * this.m_impulse * this.m_J.linearB.x),
        (e.m_linearVelocity.y +=
          e.m_invMass * this.m_impulse * this.m_J.linearB.y),
        (e.m_angularVelocity += e.m_invI * this.m_impulse * this.m_J.angularB))
      : (this.m_impulse = 0);
  };
  P.prototype.SolveVelocityConstraints = function (a) {
    a = this.m_bodyA;
    var d = this.m_bodyB,
      c = this.m_J.Compute(
        a.m_linearVelocity,
        a.m_angularVelocity,
        d.m_linearVelocity,
        d.m_angularVelocity
      ),
      c = -this.m_mass * c;
    this.m_impulse += c;
    a.m_linearVelocity.x += a.m_invMass * c * this.m_J.linearA.x;
    a.m_linearVelocity.y += a.m_invMass * c * this.m_J.linearA.y;
    a.m_angularVelocity += a.m_invI * c * this.m_J.angularA;
    d.m_linearVelocity.x += d.m_invMass * c * this.m_J.linearB.x;
    d.m_linearVelocity.y += d.m_invMass * c * this.m_J.linearB.y;
    d.m_angularVelocity += d.m_invI * c * this.m_J.angularB;
  };
  P.prototype.SolvePositionConstraints = function (a) {
    a = this.m_bodyA;
    var d = this.m_bodyB;
    var c = this.m_revolute1
      ? this.m_revolute1.GetJointAngle()
      : this.m_prismatic1.GetJointTranslation();
    var f = this.m_revolute2
      ? this.m_revolute2.GetJointAngle()
      : this.m_prismatic2.GetJointTranslation();
    c = -this.m_mass * (this.m_constant - (c + this.m_ratio * f));
    a.m_sweep.c.x += a.m_invMass * c * this.m_J.linearA.x;
    a.m_sweep.c.y += a.m_invMass * c * this.m_J.linearA.y;
    a.m_sweep.a += a.m_invI * c * this.m_J.angularA;
    d.m_sweep.c.x += d.m_invMass * c * this.m_J.linearB.x;
    d.m_sweep.c.y += d.m_invMass * c * this.m_J.linearB.y;
    d.m_sweep.a += d.m_invI * c * this.m_J.angularB;
    a.SynchronizeTransform();
    d.SynchronizeTransform();
    return 0 < n.b2_linearSlop;
  };
  Box2D.inherit(H, Box2D.Dynamics.Joints.b2JointDef);
  H.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
  H.b2GearJointDef = function () {
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
  };
  H.prototype.b2GearJointDef = function () {
    this.__super.b2JointDef.call(this);
    this.type = y.e_gearJoint;
    this.joint2 = this.joint1 = null;
    this.ratio = 1;
  };
  L.b2Jacobian = function () {
    this.linearA = new h();
    this.linearB = new h();
  };
  L.prototype.SetZero = function () {
    this.linearA.SetZero();
    this.angularA = 0;
    this.linearB.SetZero();
    this.angularB = 0;
  };
  L.prototype.Set = function (a, d, c, f) {
    void 0 === d && (d = 0);
    void 0 === f && (f = 0);
    this.linearA.SetV(a);
    this.angularA = d;
    this.linearB.SetV(c);
    this.angularB = f;
  };
  L.prototype.Compute = function (a, d, c, f) {
    void 0 === d && (d = 0);
    void 0 === f && (f = 0);
    return (
      this.linearA.x * a.x +
      this.linearA.y * a.y +
      this.angularA * d +
      (this.linearB.x * c.x + this.linearB.y * c.y) +
      this.angularB * f
    );
  };
  y.b2Joint = function () {
    this.m_edgeA = new u();
    this.m_edgeB = new u();
    this.m_localCenterA = new h();
    this.m_localCenterB = new h();
  };
  y.prototype.GetType = function () {
    return this.m_type;
  };
  y.prototype.GetAnchorA = function () {
    return null;
  };
  y.prototype.GetAnchorB = function () {
    return null;
  };
  y.prototype.GetReactionForce = function (a) {
    return null;
  };
  y.prototype.GetReactionTorque = function (a) {
    return 0;
  };
  y.prototype.GetBodyA = function () {
    return this.m_bodyA;
  };
  y.prototype.GetBodyB = function () {
    return this.m_bodyB;
  };
  y.prototype.GetNext = function () {
    return this.m_next;
  };
  y.prototype.GetUserData = function () {
    return this.m_userData;
  };
  y.prototype.SetUserData = function (a) {
    this.m_userData = a;
  };
  y.prototype.IsActive = function () {
    return this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
  };
  y.Create = function (a, d) {
    var c = null;
    switch (a.type) {
      case y.e_distanceJoint:
        c = new A(a instanceof e ? a : null);
        break;
      case y.e_mouseJoint:
        c = new E(a instanceof M ? a : null);
        break;
      case y.e_prismaticJoint:
        c = new K(a instanceof N ? a : null);
        break;
      case y.e_revoluteJoint:
        c = new F(a instanceof I ? a : null);
        break;
      case y.e_pulleyJoint:
        c = new D(a instanceof R ? a : null);
        break;
      case y.e_gearJoint:
        c = new P(a instanceof H ? a : null);
        break;
      case y.e_lineJoint:
        c = new g(a instanceof C ? a : null);
        break;
      case y.e_weldJoint:
        c = new S(a instanceof O ? a : null);
        break;
      case y.e_frictionJoint:
        c = new w(a instanceof G ? a : null);
    }
    return c;
  };
  y.Destroy = function (a, d) {};
  y.prototype.b2Joint = function (a) {
    n.b2Assert(a.bodyA != a.bodyB);
    this.m_type = a.type;
    this.m_next = this.m_prev = null;
    this.m_bodyA = a.bodyA;
    this.m_bodyB = a.bodyB;
    this.m_collideConnected = a.collideConnected;
    this.m_islandFlag = !1;
    this.m_userData = a.userData;
  };
  y.prototype.InitVelocityConstraints = function (a) {};
  y.prototype.SolveVelocityConstraints = function (a) {};
  y.prototype.FinalizeVelocityConstraints = function () {};
  y.prototype.SolvePositionConstraints = function (a) {
    return !1;
  };
  Box2D.postDefs.push(function () {
    Box2D.Dynamics.Joints.b2Joint.e_unknownJoint = 0;
    Box2D.Dynamics.Joints.b2Joint.e_revoluteJoint = 1;
    Box2D.Dynamics.Joints.b2Joint.e_prismaticJoint = 2;
    Box2D.Dynamics.Joints.b2Joint.e_distanceJoint = 3;
    Box2D.Dynamics.Joints.b2Joint.e_pulleyJoint = 4;
    Box2D.Dynamics.Joints.b2Joint.e_mouseJoint = 5;
    Box2D.Dynamics.Joints.b2Joint.e_gearJoint = 6;
    Box2D.Dynamics.Joints.b2Joint.e_lineJoint = 7;
    Box2D.Dynamics.Joints.b2Joint.e_weldJoint = 8;
    Box2D.Dynamics.Joints.b2Joint.e_frictionJoint = 9;
    Box2D.Dynamics.Joints.b2Joint.e_inactiveLimit = 0;
    Box2D.Dynamics.Joints.b2Joint.e_atLowerLimit = 1;
    Box2D.Dynamics.Joints.b2Joint.e_atUpperLimit = 2;
    Box2D.Dynamics.Joints.b2Joint.e_equalLimits = 3;
  });
  Q.b2JointDef = function () {};
  Q.prototype.b2JointDef = function () {
    this.type = y.e_unknownJoint;
    this.bodyB = this.bodyA = this.userData = null;
    this.collideConnected = !1;
  };
  u.b2JointEdge = function () {};
  Box2D.inherit(g, Box2D.Dynamics.Joints.b2Joint);
  g.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
  g.b2LineJoint = function () {
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
    this.m_localAnchor1 = new h();
    this.m_localAnchor2 = new h();
    this.m_localXAxis1 = new h();
    this.m_localYAxis1 = new h();
    this.m_axis = new h();
    this.m_perp = new h();
    this.m_K = new x();
    this.m_impulse = new h();
  };
  g.prototype.GetAnchorA = function () {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
  };
  g.prototype.GetAnchorB = function () {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
  };
  g.prototype.GetReactionForce = function (a) {
    void 0 === a && (a = 0);
    return new h(
      a *
        (this.m_impulse.x * this.m_perp.x +
          (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.x),
      a *
        (this.m_impulse.x * this.m_perp.y +
          (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.y)
    );
  };
  g.prototype.GetReactionTorque = function (a) {
    void 0 === a && (a = 0);
    return a * this.m_impulse.y;
  };
  g.prototype.GetJointTranslation = function () {
    var a = this.m_bodyA,
      d = this.m_bodyB,
      c = a.GetWorldPoint(this.m_localAnchor1),
      f = d.GetWorldPoint(this.m_localAnchor2),
      d = f.x - c.x,
      c = f.y - c.y,
      a = a.GetWorldVector(this.m_localXAxis1);
    return a.x * d + a.y * c;
  };
  g.prototype.GetJointSpeed = function () {
    var a = this.m_bodyA,
      d = this.m_bodyB;
    var c = a.m_xf.R;
    var f = this.m_localAnchor1.x - a.m_sweep.localCenter.x,
      e = this.m_localAnchor1.y - a.m_sweep.localCenter.y,
      g = c.col1.x * f + c.col2.x * e,
      e = c.col1.y * f + c.col2.y * e,
      f = g;
    c = d.m_xf.R;
    var h = this.m_localAnchor2.x - d.m_sweep.localCenter.x,
      b = this.m_localAnchor2.y - d.m_sweep.localCenter.y,
      g = c.col1.x * h + c.col2.x * b,
      b = c.col1.y * h + c.col2.y * b,
      h = g;
    c = d.m_sweep.c.x + h - (a.m_sweep.c.x + f);
    var g = d.m_sweep.c.y + b - (a.m_sweep.c.y + e),
      k = a.GetWorldVector(this.m_localXAxis1),
      l = a.m_linearVelocity,
      m = d.m_linearVelocity,
      a = a.m_angularVelocity,
      d = d.m_angularVelocity;
    return (
      c * -a * k.y +
      g * a * k.x +
      (k.x * (m.x + -d * b - l.x - -a * e) + k.y * (m.y + d * h - l.y - a * f))
    );
  };
  g.prototype.IsLimitEnabled = function () {
    return this.m_enableLimit;
  };
  g.prototype.EnableLimit = function (a) {
    this.m_bodyA.SetAwake(!0);
    this.m_bodyB.SetAwake(!0);
    this.m_enableLimit = a;
  };
  g.prototype.GetLowerLimit = function () {
    return this.m_lowerTranslation;
  };
  g.prototype.GetUpperLimit = function () {
    return this.m_upperTranslation;
  };
  g.prototype.SetLimits = function (a, d) {
    void 0 === a && (a = 0);
    void 0 === d && (d = 0);
    this.m_bodyA.SetAwake(!0);
    this.m_bodyB.SetAwake(!0);
    this.m_lowerTranslation = a;
    this.m_upperTranslation = d;
  };
  g.prototype.IsMotorEnabled = function () {
    return this.m_enableMotor;
  };
  g.prototype.EnableMotor = function (a) {
    this.m_bodyA.SetAwake(!0);
    this.m_bodyB.SetAwake(!0);
    this.m_enableMotor = a;
  };
  g.prototype.SetMotorSpeed = function (a) {
    void 0 === a && (a = 0);
    this.m_bodyA.SetAwake(!0);
    this.m_bodyB.SetAwake(!0);
    this.m_motorSpeed = a;
  };
  g.prototype.GetMotorSpeed = function () {
    return this.m_motorSpeed;
  };
  g.prototype.SetMaxMotorForce = function (a) {
    void 0 === a && (a = 0);
    this.m_bodyA.SetAwake(!0);
    this.m_bodyB.SetAwake(!0);
    this.m_maxMotorForce = a;
  };
  g.prototype.GetMaxMotorForce = function () {
    return this.m_maxMotorForce;
  };
  g.prototype.GetMotorForce = function () {
    return this.m_motorImpulse;
  };
  g.prototype.b2LineJoint = function (a) {
    this.__super.b2Joint.call(this, a);
    this.m_localAnchor1.SetV(a.localAnchorA);
    this.m_localAnchor2.SetV(a.localAnchorB);
    this.m_localXAxis1.SetV(a.localAxisA);
    this.m_localYAxis1.x = -this.m_localXAxis1.y;
    this.m_localYAxis1.y = this.m_localXAxis1.x;
    this.m_impulse.SetZero();
    this.m_motorImpulse = this.m_motorMass = 0;
    this.m_lowerTranslation = a.lowerTranslation;
    this.m_upperTranslation = a.upperTranslation;
    this.m_maxMotorForce = a.maxMotorForce;
    this.m_motorSpeed = a.motorSpeed;
    this.m_enableLimit = a.enableLimit;
    this.m_enableMotor = a.enableMotor;
    this.m_limitState = y.e_inactiveLimit;
    this.m_axis.SetZero();
    this.m_perp.SetZero();
  };
  g.prototype.InitVelocityConstraints = function (a) {
    var d = this.m_bodyA,
      e = this.m_bodyB;
    this.m_localCenterA.SetV(d.GetLocalCenter());
    this.m_localCenterB.SetV(e.GetLocalCenter());
    var f = d.GetTransform();
    e.GetTransform();
    var g = d.m_xf.R;
    var h = this.m_localAnchor1.x - this.m_localCenterA.x,
      l = this.m_localAnchor1.y - this.m_localCenterA.y;
    var b = g.col1.x * h + g.col2.x * l;
    l = g.col1.y * h + g.col2.y * l;
    h = b;
    g = e.m_xf.R;
    var k = this.m_localAnchor2.x - this.m_localCenterB.x,
      q = this.m_localAnchor2.y - this.m_localCenterB.y;
    b = g.col1.x * k + g.col2.x * q;
    q = g.col1.y * k + g.col2.y * q;
    k = b;
    g = e.m_sweep.c.x + k - d.m_sweep.c.x - h;
    b = e.m_sweep.c.y + q - d.m_sweep.c.y - l;
    this.m_invMassA = d.m_invMass;
    this.m_invMassB = e.m_invMass;
    this.m_invIA = d.m_invI;
    this.m_invIB = e.m_invI;
    this.m_axis.SetV(c.MulMV(f.R, this.m_localXAxis1));
    this.m_a1 = (g + h) * this.m_axis.y - (b + l) * this.m_axis.x;
    this.m_a2 = k * this.m_axis.y - q * this.m_axis.x;
    this.m_motorMass =
      this.m_invMassA +
      this.m_invMassB +
      this.m_invIA * this.m_a1 * this.m_a1 +
      this.m_invIB * this.m_a2 * this.m_a2;
    this.m_motorMass =
      this.m_motorMass > Number.MIN_VALUE ? 1 / this.m_motorMass : 0;
    this.m_perp.SetV(c.MulMV(f.R, this.m_localYAxis1));
    this.m_s1 = (g + h) * this.m_perp.y - (b + l) * this.m_perp.x;
    this.m_s2 = k * this.m_perp.y - q * this.m_perp.x;
    f = this.m_invMassA;
    h = this.m_invMassB;
    l = this.m_invIA;
    k = this.m_invIB;
    this.m_K.col1.x =
      f + h + l * this.m_s1 * this.m_s1 + k * this.m_s2 * this.m_s2;
    this.m_K.col1.y = l * this.m_s1 * this.m_a1 + k * this.m_s2 * this.m_a2;
    this.m_K.col2.x = this.m_K.col1.y;
    this.m_K.col2.y =
      f + h + l * this.m_a1 * this.m_a1 + k * this.m_a2 * this.m_a2;
    this.m_enableLimit
      ? ((g = this.m_axis.x * g + this.m_axis.y * b),
        c.Abs(this.m_upperTranslation - this.m_lowerTranslation) <
        2 * n.b2_linearSlop
          ? (this.m_limitState = y.e_equalLimits)
          : g <= this.m_lowerTranslation
          ? this.m_limitState != y.e_atLowerLimit &&
            ((this.m_limitState = y.e_atLowerLimit), (this.m_impulse.y = 0))
          : g >= this.m_upperTranslation
          ? this.m_limitState != y.e_atUpperLimit &&
            ((this.m_limitState = y.e_atUpperLimit), (this.m_impulse.y = 0))
          : ((this.m_limitState = y.e_inactiveLimit), (this.m_impulse.y = 0)))
      : (this.m_limitState = y.e_inactiveLimit);
    0 == this.m_enableMotor && (this.m_motorImpulse = 0);
    a.warmStarting
      ? ((this.m_impulse.x *= a.dtRatio),
        (this.m_impulse.y *= a.dtRatio),
        (this.m_motorImpulse *= a.dtRatio),
        (a =
          this.m_impulse.x * this.m_perp.x +
          (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.x),
        (g =
          this.m_impulse.x * this.m_perp.y +
          (this.m_motorImpulse + this.m_impulse.y) * this.m_axis.y),
        (b =
          this.m_impulse.x * this.m_s1 +
          (this.m_motorImpulse + this.m_impulse.y) * this.m_a1),
        (f =
          this.m_impulse.x * this.m_s2 +
          (this.m_motorImpulse + this.m_impulse.y) * this.m_a2),
        (d.m_linearVelocity.x -= this.m_invMassA * a),
        (d.m_linearVelocity.y -= this.m_invMassA * g),
        (d.m_angularVelocity -= this.m_invIA * b),
        (e.m_linearVelocity.x += this.m_invMassB * a),
        (e.m_linearVelocity.y += this.m_invMassB * g),
        (e.m_angularVelocity += this.m_invIB * f))
      : (this.m_impulse.SetZero(), (this.m_motorImpulse = 0));
  };
  g.prototype.SolveVelocityConstraints = function (a) {
    var d = this.m_bodyA,
      e = this.m_bodyB,
      f = d.m_linearVelocity,
      g = d.m_angularVelocity,
      l = e.m_linearVelocity,
      n = e.m_angularVelocity;
    if (this.m_enableMotor && this.m_limitState != y.e_equalLimits) {
      var b =
        this.m_motorMass *
        (this.m_motorSpeed -
          (this.m_axis.x * (l.x - f.x) +
            this.m_axis.y * (l.y - f.y) +
            this.m_a2 * n -
            this.m_a1 * g));
      var k = this.m_motorImpulse;
      a = a.dt * this.m_maxMotorForce;
      this.m_motorImpulse = c.Clamp(this.m_motorImpulse + b, -a, a);
      b = this.m_motorImpulse - k;
      k = b * this.m_axis.x;
      a = b * this.m_axis.y;
      var q = b * this.m_a1;
      b *= this.m_a2;
      f.x -= this.m_invMassA * k;
      f.y -= this.m_invMassA * a;
      g -= this.m_invIA * q;
      l.x += this.m_invMassB * k;
      l.y += this.m_invMassB * a;
      n += this.m_invIB * b;
    }
    a =
      this.m_perp.x * (l.x - f.x) +
      this.m_perp.y * (l.y - f.y) +
      this.m_s2 * n -
      this.m_s1 * g;
    this.m_enableLimit && this.m_limitState != y.e_inactiveLimit
      ? ((q =
          this.m_axis.x * (l.x - f.x) +
          this.m_axis.y * (l.y - f.y) +
          this.m_a2 * n -
          this.m_a1 * g),
        (k = this.m_impulse.Copy()),
        (b = this.m_K.Solve(new h(), -a, -q)),
        this.m_impulse.Add(b),
        this.m_limitState == y.e_atLowerLimit
          ? (this.m_impulse.y = c.Max(this.m_impulse.y, 0))
          : this.m_limitState == y.e_atUpperLimit &&
            (this.m_impulse.y = c.Min(this.m_impulse.y, 0)),
        (a = -a - (this.m_impulse.y - k.y) * this.m_K.col2.x),
        (this.m_impulse.x =
          0 != this.m_K.col1.x ? a / this.m_K.col1.x + k.x : k.x),
        (b.x = this.m_impulse.x - k.x),
        (b.y = this.m_impulse.y - k.y),
        (k = b.x * this.m_perp.x + b.y * this.m_axis.x),
        (a = b.x * this.m_perp.y + b.y * this.m_axis.y),
        (q = b.x * this.m_s1 + b.y * this.m_a1),
        (b = b.x * this.m_s2 + b.y * this.m_a2))
      : ((b = 0 != this.m_K.col1.x ? -a / this.m_K.col1.x : 0),
        (this.m_impulse.x += b),
        (k = b * this.m_perp.x),
        (a = b * this.m_perp.y),
        (q = b * this.m_s1),
        (b *= this.m_s2));
    f.x -= this.m_invMassA * k;
    f.y -= this.m_invMassA * a;
    g -= this.m_invIA * q;
    l.x += this.m_invMassB * k;
    l.y += this.m_invMassB * a;
    n += this.m_invIB * b;
    d.m_linearVelocity.SetV(f);
    d.m_angularVelocity = g;
    e.m_linearVelocity.SetV(l);
    e.m_angularVelocity = n;
  };
  g.prototype.SolvePositionConstraints = function (a) {
    a = this.m_bodyA;
    var d = this.m_bodyB,
      e = a.m_sweep.c,
      f = a.m_sweep.a,
      g = d.m_sweep.c,
      l = d.m_sweep.a,
      t = 0;
    var b = !1;
    var k = 0,
      q = x.FromAngle(f);
    var m = x.FromAngle(l);
    var p = q;
    var u = this.m_localAnchor1.x - this.m_localCenterA.x;
    var w = this.m_localAnchor1.y - this.m_localCenterA.y;
    var y = p.col1.x * u + p.col2.x * w;
    w = p.col1.y * u + p.col2.y * w;
    u = y;
    p = m;
    m = this.m_localAnchor2.x - this.m_localCenterB.x;
    var A = this.m_localAnchor2.y - this.m_localCenterB.y;
    y = p.col1.x * m + p.col2.x * A;
    A = p.col1.y * m + p.col2.y * A;
    m = y;
    p = g.x + m - e.x - u;
    y = g.y + A - e.y - w;
    if (this.m_enableLimit) {
      this.m_axis = c.MulMV(q, this.m_localXAxis1);
      this.m_a1 = (p + u) * this.m_axis.y - (y + w) * this.m_axis.x;
      this.m_a2 = m * this.m_axis.y - A * this.m_axis.x;
      var C = this.m_axis.x * p + this.m_axis.y * y;
      c.Abs(this.m_upperTranslation - this.m_lowerTranslation) <
      2 * n.b2_linearSlop
        ? ((k = c.Clamp(
            C,
            -n.b2_maxLinearCorrection,
            n.b2_maxLinearCorrection
          )),
          (t = c.Abs(C)),
          (b = !0))
        : C <= this.m_lowerTranslation
        ? ((k = c.Clamp(
            C - this.m_lowerTranslation + n.b2_linearSlop,
            -n.b2_maxLinearCorrection,
            0
          )),
          (t = this.m_lowerTranslation - C),
          (b = !0))
        : C >= this.m_upperTranslation &&
          ((k = c.Clamp(
            C - this.m_upperTranslation + n.b2_linearSlop,
            0,
            n.b2_maxLinearCorrection
          )),
          (t = C - this.m_upperTranslation),
          (b = !0));
    }
    this.m_perp = c.MulMV(q, this.m_localYAxis1);
    this.m_s1 = (p + u) * this.m_perp.y - (y + w) * this.m_perp.x;
    this.m_s2 = m * this.m_perp.y - A * this.m_perp.x;
    q = new h();
    u = this.m_perp.x * p + this.m_perp.y * y;
    t = c.Max(t, c.Abs(u));
    b
      ? ((b = this.m_invMassA),
        (w = this.m_invMassB),
        (m = this.m_invIA),
        (A = this.m_invIB),
        (this.m_K.col1.x =
          b + w + m * this.m_s1 * this.m_s1 + A * this.m_s2 * this.m_s2),
        (this.m_K.col1.y =
          m * this.m_s1 * this.m_a1 + A * this.m_s2 * this.m_a2),
        (this.m_K.col2.x = this.m_K.col1.y),
        (this.m_K.col2.y =
          b + w + m * this.m_a1 * this.m_a1 + A * this.m_a2 * this.m_a2),
        this.m_K.Solve(q, -u, -k))
      : ((b = this.m_invMassA),
        (w = this.m_invMassB),
        (m = this.m_invIA),
        (A = this.m_invIB),
        (k = b + w + m * this.m_s1 * this.m_s1 + A * this.m_s2 * this.m_s2),
        (q.x = 0 != k ? -u / k : 0),
        (q.y = 0));
    k = q.x * this.m_perp.x + q.y * this.m_axis.x;
    b = q.x * this.m_perp.y + q.y * this.m_axis.y;
    u = q.x * this.m_s1 + q.y * this.m_a1;
    q = q.x * this.m_s2 + q.y * this.m_a2;
    e.x -= this.m_invMassA * k;
    e.y -= this.m_invMassA * b;
    f -= this.m_invIA * u;
    g.x += this.m_invMassB * k;
    g.y += this.m_invMassB * b;
    l += this.m_invIB * q;
    a.m_sweep.a = f;
    d.m_sweep.a = l;
    a.SynchronizeTransform();
    d.SynchronizeTransform();
    return t <= n.b2_linearSlop && 0 <= n.b2_angularSlop;
  };
  Box2D.inherit(C, Box2D.Dynamics.Joints.b2JointDef);
  C.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
  C.b2LineJointDef = function () {
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
    this.localAnchorA = new h();
    this.localAnchorB = new h();
    this.localAxisA = new h();
  };
  C.prototype.b2LineJointDef = function () {
    this.__super.b2JointDef.call(this);
    this.type = y.e_lineJoint;
    this.localAxisA.Set(1, 0);
    this.enableLimit = !1;
    this.upperTranslation = this.lowerTranslation = 0;
    this.enableMotor = !1;
    this.motorSpeed = this.maxMotorForce = 0;
  };
  C.prototype.Initialize = function (a, d, c, f) {
    this.bodyA = a;
    this.bodyB = d;
    this.localAnchorA = this.bodyA.GetLocalPoint(c);
    this.localAnchorB = this.bodyB.GetLocalPoint(c);
    this.localAxisA = this.bodyA.GetLocalVector(f);
  };
  Box2D.inherit(E, Box2D.Dynamics.Joints.b2Joint);
  E.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
  E.b2MouseJoint = function () {
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
    this.K = new x();
    this.K1 = new x();
    this.K2 = new x();
    this.m_localAnchor = new h();
    this.m_target = new h();
    this.m_impulse = new h();
    this.m_mass = new x();
    this.m_C = new h();
  };
  E.prototype.GetAnchorA = function () {
    return this.m_target;
  };
  E.prototype.GetAnchorB = function () {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchor);
  };
  E.prototype.GetReactionForce = function (a) {
    void 0 === a && (a = 0);
    return new h(a * this.m_impulse.x, a * this.m_impulse.y);
  };
  E.prototype.GetReactionTorque = function (a) {
    return 0;
  };
  E.prototype.GetTarget = function () {
    return this.m_target;
  };
  E.prototype.SetTarget = function (a) {
    0 == this.m_bodyB.IsAwake() && this.m_bodyB.SetAwake(!0);
    this.m_target = a;
  };
  E.prototype.GetMaxForce = function () {
    return this.m_maxForce;
  };
  E.prototype.SetMaxForce = function (a) {
    void 0 === a && (a = 0);
    this.m_maxForce = a;
  };
  E.prototype.GetFrequency = function () {
    return this.m_frequencyHz;
  };
  E.prototype.SetFrequency = function (a) {
    void 0 === a && (a = 0);
    this.m_frequencyHz = a;
  };
  E.prototype.GetDampingRatio = function () {
    return this.m_dampingRatio;
  };
  E.prototype.SetDampingRatio = function (a) {
    void 0 === a && (a = 0);
    this.m_dampingRatio = a;
  };
  E.prototype.b2MouseJoint = function (a) {
    this.__super.b2Joint.call(this, a);
    this.m_target.SetV(a.target);
    var d = this.m_target.x - this.m_bodyB.m_xf.position.x,
      c = this.m_target.y - this.m_bodyB.m_xf.position.y,
      f = this.m_bodyB.m_xf.R;
    this.m_localAnchor.x = d * f.col1.x + c * f.col1.y;
    this.m_localAnchor.y = d * f.col2.x + c * f.col2.y;
    this.m_maxForce = a.maxForce;
    this.m_impulse.SetZero();
    this.m_frequencyHz = a.frequencyHz;
    this.m_dampingRatio = a.dampingRatio;
    this.m_gamma = this.m_beta = 0;
  };
  E.prototype.InitVelocityConstraints = function (a) {
    var d = this.m_bodyB,
      c = d.GetMass(),
      f = 2 * Math.PI * this.m_frequencyHz,
      e = c * f * f;
    this.m_gamma = a.dt * (2 * c * this.m_dampingRatio * f + a.dt * e);
    this.m_gamma = 0 != this.m_gamma ? 1 / this.m_gamma : 0;
    this.m_beta = a.dt * e * this.m_gamma;
    var e = d.m_xf.R,
      c = this.m_localAnchor.x - d.m_sweep.localCenter.x,
      f = this.m_localAnchor.y - d.m_sweep.localCenter.y,
      g = e.col1.x * c + e.col2.x * f,
      f = e.col1.y * c + e.col2.y * f,
      c = g,
      e = d.m_invMass,
      g = d.m_invI;
    this.K1.col1.x = e;
    this.K1.col2.x = 0;
    this.K1.col1.y = 0;
    this.K1.col2.y = e;
    this.K2.col1.x = g * f * f;
    this.K2.col2.x = -g * c * f;
    this.K2.col1.y = -g * c * f;
    this.K2.col2.y = g * c * c;
    this.K.SetM(this.K1);
    this.K.AddM(this.K2);
    this.K.col1.x += this.m_gamma;
    this.K.col2.y += this.m_gamma;
    this.K.GetInverse(this.m_mass);
    this.m_C.x = d.m_sweep.c.x + c - this.m_target.x;
    this.m_C.y = d.m_sweep.c.y + f - this.m_target.y;
    d.m_angularVelocity *= 0.98;
    this.m_impulse.x *= a.dtRatio;
    this.m_impulse.y *= a.dtRatio;
    d.m_linearVelocity.x += e * this.m_impulse.x;
    d.m_linearVelocity.y += e * this.m_impulse.y;
    d.m_angularVelocity += g * (c * this.m_impulse.y - f * this.m_impulse.x);
  };
  E.prototype.SolveVelocityConstraints = function (a) {
    var d = this.m_bodyB;
    var c = d.m_xf.R;
    var f = this.m_localAnchor.x - d.m_sweep.localCenter.x,
      e = this.m_localAnchor.y - d.m_sweep.localCenter.y;
    var g = c.col1.x * f + c.col2.x * e;
    e = c.col1.y * f + c.col2.y * e;
    f = g;
    g = d.m_linearVelocity.x + -d.m_angularVelocity * e;
    var h = d.m_linearVelocity.y + d.m_angularVelocity * f;
    c = this.m_mass;
    g = g + this.m_beta * this.m_C.x + this.m_gamma * this.m_impulse.x;
    var b = h + this.m_beta * this.m_C.y + this.m_gamma * this.m_impulse.y;
    h = -(c.col1.x * g + c.col2.x * b);
    b = -(c.col1.y * g + c.col2.y * b);
    c = this.m_impulse.x;
    g = this.m_impulse.y;
    this.m_impulse.x += h;
    this.m_impulse.y += b;
    a = a.dt * this.m_maxForce;
    this.m_impulse.LengthSquared() > a * a &&
      this.m_impulse.Multiply(a / this.m_impulse.Length());
    h = this.m_impulse.x - c;
    b = this.m_impulse.y - g;
    d.m_linearVelocity.x += d.m_invMass * h;
    d.m_linearVelocity.y += d.m_invMass * b;
    d.m_angularVelocity += d.m_invI * (f * b - e * h);
  };
  E.prototype.SolvePositionConstraints = function (a) {
    return !0;
  };
  Box2D.inherit(M, Box2D.Dynamics.Joints.b2JointDef);
  M.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
  M.b2MouseJointDef = function () {
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
    this.target = new h();
  };
  M.prototype.b2MouseJointDef = function () {
    this.__super.b2JointDef.call(this);
    this.type = y.e_mouseJoint;
    this.maxForce = 0;
    this.frequencyHz = 5;
    this.dampingRatio = 0.7;
  };
  Box2D.inherit(K, Box2D.Dynamics.Joints.b2Joint);
  K.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
  K.b2PrismaticJoint = function () {
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
    this.m_localAnchor1 = new h();
    this.m_localAnchor2 = new h();
    this.m_localXAxis1 = new h();
    this.m_localYAxis1 = new h();
    this.m_axis = new h();
    this.m_perp = new h();
    this.m_K = new t();
    this.m_impulse = new l();
  };
  K.prototype.GetAnchorA = function () {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
  };
  K.prototype.GetAnchorB = function () {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
  };
  K.prototype.GetReactionForce = function (a) {
    void 0 === a && (a = 0);
    return new h(
      a *
        (this.m_impulse.x * this.m_perp.x +
          (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.x),
      a *
        (this.m_impulse.x * this.m_perp.y +
          (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.y)
    );
  };
  K.prototype.GetReactionTorque = function (a) {
    void 0 === a && (a = 0);
    return a * this.m_impulse.y;
  };
  K.prototype.GetJointTranslation = function () {
    var a = this.m_bodyA,
      d = this.m_bodyB,
      c = a.GetWorldPoint(this.m_localAnchor1),
      f = d.GetWorldPoint(this.m_localAnchor2),
      d = f.x - c.x,
      c = f.y - c.y,
      a = a.GetWorldVector(this.m_localXAxis1);
    return a.x * d + a.y * c;
  };
  K.prototype.GetJointSpeed = function () {
    var a = this.m_bodyA,
      d = this.m_bodyB;
    var c = a.m_xf.R;
    var f = this.m_localAnchor1.x - a.m_sweep.localCenter.x,
      e = this.m_localAnchor1.y - a.m_sweep.localCenter.y,
      g = c.col1.x * f + c.col2.x * e,
      e = c.col1.y * f + c.col2.y * e,
      f = g;
    c = d.m_xf.R;
    var h = this.m_localAnchor2.x - d.m_sweep.localCenter.x,
      b = this.m_localAnchor2.y - d.m_sweep.localCenter.y,
      g = c.col1.x * h + c.col2.x * b,
      b = c.col1.y * h + c.col2.y * b,
      h = g;
    c = d.m_sweep.c.x + h - (a.m_sweep.c.x + f);
    var g = d.m_sweep.c.y + b - (a.m_sweep.c.y + e),
      k = a.GetWorldVector(this.m_localXAxis1),
      l = a.m_linearVelocity,
      m = d.m_linearVelocity,
      a = a.m_angularVelocity,
      d = d.m_angularVelocity;
    return (
      c * -a * k.y +
      g * a * k.x +
      (k.x * (m.x + -d * b - l.x - -a * e) + k.y * (m.y + d * h - l.y - a * f))
    );
  };
  K.prototype.IsLimitEnabled = function () {
    return this.m_enableLimit;
  };
  K.prototype.EnableLimit = function (a) {
    this.m_bodyA.SetAwake(!0);
    this.m_bodyB.SetAwake(!0);
    this.m_enableLimit = a;
  };
  K.prototype.GetLowerLimit = function () {
    return this.m_lowerTranslation;
  };
  K.prototype.GetUpperLimit = function () {
    return this.m_upperTranslation;
  };
  K.prototype.SetLimits = function (a, d) {
    void 0 === a && (a = 0);
    void 0 === d && (d = 0);
    this.m_bodyA.SetAwake(!0);
    this.m_bodyB.SetAwake(!0);
    this.m_lowerTranslation = a;
    this.m_upperTranslation = d;
  };
  K.prototype.IsMotorEnabled = function () {
    return this.m_enableMotor;
  };
  K.prototype.EnableMotor = function (a) {
    this.m_bodyA.SetAwake(!0);
    this.m_bodyB.SetAwake(!0);
    this.m_enableMotor = a;
  };
  K.prototype.SetMotorSpeed = function (a) {
    void 0 === a && (a = 0);
    this.m_bodyA.SetAwake(!0);
    this.m_bodyB.SetAwake(!0);
    this.m_motorSpeed = a;
  };
  K.prototype.GetMotorSpeed = function () {
    return this.m_motorSpeed;
  };
  K.prototype.SetMaxMotorForce = function (a) {
    void 0 === a && (a = 0);
    this.m_bodyA.SetAwake(!0);
    this.m_bodyB.SetAwake(!0);
    this.m_maxMotorForce = a;
  };
  K.prototype.GetMotorForce = function () {
    return this.m_motorImpulse;
  };
  K.prototype.b2PrismaticJoint = function (a) {
    this.__super.b2Joint.call(this, a);
    this.m_localAnchor1.SetV(a.localAnchorA);
    this.m_localAnchor2.SetV(a.localAnchorB);
    this.m_localXAxis1.SetV(a.localAxisA);
    this.m_localYAxis1.x = -this.m_localXAxis1.y;
    this.m_localYAxis1.y = this.m_localXAxis1.x;
    this.m_refAngle = a.referenceAngle;
    this.m_impulse.SetZero();
    this.m_motorImpulse = this.m_motorMass = 0;
    this.m_lowerTranslation = a.lowerTranslation;
    this.m_upperTranslation = a.upperTranslation;
    this.m_maxMotorForce = a.maxMotorForce;
    this.m_motorSpeed = a.motorSpeed;
    this.m_enableLimit = a.enableLimit;
    this.m_enableMotor = a.enableMotor;
    this.m_limitState = y.e_inactiveLimit;
    this.m_axis.SetZero();
    this.m_perp.SetZero();
  };
  K.prototype.InitVelocityConstraints = function (a) {
    var d = this.m_bodyA,
      e = this.m_bodyB;
    this.m_localCenterA.SetV(d.GetLocalCenter());
    this.m_localCenterB.SetV(e.GetLocalCenter());
    var f = d.GetTransform();
    e.GetTransform();
    var g = d.m_xf.R;
    var h = this.m_localAnchor1.x - this.m_localCenterA.x,
      l = this.m_localAnchor1.y - this.m_localCenterA.y;
    var b = g.col1.x * h + g.col2.x * l;
    l = g.col1.y * h + g.col2.y * l;
    h = b;
    g = e.m_xf.R;
    var k = this.m_localAnchor2.x - this.m_localCenterB.x,
      q = this.m_localAnchor2.y - this.m_localCenterB.y;
    b = g.col1.x * k + g.col2.x * q;
    q = g.col1.y * k + g.col2.y * q;
    k = b;
    g = e.m_sweep.c.x + k - d.m_sweep.c.x - h;
    b = e.m_sweep.c.y + q - d.m_sweep.c.y - l;
    this.m_invMassA = d.m_invMass;
    this.m_invMassB = e.m_invMass;
    this.m_invIA = d.m_invI;
    this.m_invIB = e.m_invI;
    this.m_axis.SetV(c.MulMV(f.R, this.m_localXAxis1));
    this.m_a1 = (g + h) * this.m_axis.y - (b + l) * this.m_axis.x;
    this.m_a2 = k * this.m_axis.y - q * this.m_axis.x;
    this.m_motorMass =
      this.m_invMassA +
      this.m_invMassB +
      this.m_invIA * this.m_a1 * this.m_a1 +
      this.m_invIB * this.m_a2 * this.m_a2;
    this.m_motorMass > Number.MIN_VALUE &&
      (this.m_motorMass = 1 / this.m_motorMass);
    this.m_perp.SetV(c.MulMV(f.R, this.m_localYAxis1));
    this.m_s1 = (g + h) * this.m_perp.y - (b + l) * this.m_perp.x;
    this.m_s2 = k * this.m_perp.y - q * this.m_perp.x;
    f = this.m_invMassA;
    h = this.m_invMassB;
    l = this.m_invIA;
    k = this.m_invIB;
    this.m_K.col1.x =
      f + h + l * this.m_s1 * this.m_s1 + k * this.m_s2 * this.m_s2;
    this.m_K.col1.y = l * this.m_s1 + k * this.m_s2;
    this.m_K.col1.z = l * this.m_s1 * this.m_a1 + k * this.m_s2 * this.m_a2;
    this.m_K.col2.x = this.m_K.col1.y;
    this.m_K.col2.y = l + k;
    this.m_K.col2.z = l * this.m_a1 + k * this.m_a2;
    this.m_K.col3.x = this.m_K.col1.z;
    this.m_K.col3.y = this.m_K.col2.z;
    this.m_K.col3.z =
      f + h + l * this.m_a1 * this.m_a1 + k * this.m_a2 * this.m_a2;
    this.m_enableLimit
      ? ((g = this.m_axis.x * g + this.m_axis.y * b),
        c.Abs(this.m_upperTranslation - this.m_lowerTranslation) <
        2 * n.b2_linearSlop
          ? (this.m_limitState = y.e_equalLimits)
          : g <= this.m_lowerTranslation
          ? this.m_limitState != y.e_atLowerLimit &&
            ((this.m_limitState = y.e_atLowerLimit), (this.m_impulse.z = 0))
          : g >= this.m_upperTranslation
          ? this.m_limitState != y.e_atUpperLimit &&
            ((this.m_limitState = y.e_atUpperLimit), (this.m_impulse.z = 0))
          : ((this.m_limitState = y.e_inactiveLimit), (this.m_impulse.z = 0)))
      : (this.m_limitState = y.e_inactiveLimit);
    0 == this.m_enableMotor && (this.m_motorImpulse = 0);
    a.warmStarting
      ? ((this.m_impulse.x *= a.dtRatio),
        (this.m_impulse.y *= a.dtRatio),
        (this.m_motorImpulse *= a.dtRatio),
        (a =
          this.m_impulse.x * this.m_perp.x +
          (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.x),
        (g =
          this.m_impulse.x * this.m_perp.y +
          (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.y),
        (b =
          this.m_impulse.x * this.m_s1 +
          this.m_impulse.y +
          (this.m_motorImpulse + this.m_impulse.z) * this.m_a1),
        (f =
          this.m_impulse.x * this.m_s2 +
          this.m_impulse.y +
          (this.m_motorImpulse + this.m_impulse.z) * this.m_a2),
        (d.m_linearVelocity.x -= this.m_invMassA * a),
        (d.m_linearVelocity.y -= this.m_invMassA * g),
        (d.m_angularVelocity -= this.m_invIA * b),
        (e.m_linearVelocity.x += this.m_invMassB * a),
        (e.m_linearVelocity.y += this.m_invMassB * g),
        (e.m_angularVelocity += this.m_invIB * f))
      : (this.m_impulse.SetZero(), (this.m_motorImpulse = 0));
  };
  K.prototype.SolveVelocityConstraints = function (a) {
    var d = this.m_bodyA,
      e = this.m_bodyB,
      f = d.m_linearVelocity,
      g = d.m_angularVelocity,
      v = e.m_linearVelocity,
      n = e.m_angularVelocity;
    if (this.m_enableMotor && this.m_limitState != y.e_equalLimits) {
      var b =
        this.m_motorMass *
        (this.m_motorSpeed -
          (this.m_axis.x * (v.x - f.x) +
            this.m_axis.y * (v.y - f.y) +
            this.m_a2 * n -
            this.m_a1 * g));
      var k = this.m_motorImpulse;
      a = a.dt * this.m_maxMotorForce;
      this.m_motorImpulse = c.Clamp(this.m_motorImpulse + b, -a, a);
      b = this.m_motorImpulse - k;
      k = b * this.m_axis.x;
      a = b * this.m_axis.y;
      var q = b * this.m_a1;
      b *= this.m_a2;
      f.x -= this.m_invMassA * k;
      f.y -= this.m_invMassA * a;
      g -= this.m_invIA * q;
      v.x += this.m_invMassB * k;
      v.y += this.m_invMassB * a;
      n += this.m_invIB * b;
    }
    q =
      this.m_perp.x * (v.x - f.x) +
      this.m_perp.y * (v.y - f.y) +
      this.m_s2 * n -
      this.m_s1 * g;
    a = n - g;
    this.m_enableLimit && this.m_limitState != y.e_inactiveLimit
      ? ((b =
          this.m_axis.x * (v.x - f.x) +
          this.m_axis.y * (v.y - f.y) +
          this.m_a2 * n -
          this.m_a1 * g),
        (k = this.m_impulse.Copy()),
        (b = this.m_K.Solve33(new l(), -q, -a, -b)),
        this.m_impulse.Add(b),
        this.m_limitState == y.e_atLowerLimit
          ? (this.m_impulse.z = c.Max(this.m_impulse.z, 0))
          : this.m_limitState == y.e_atUpperLimit &&
            (this.m_impulse.z = c.Min(this.m_impulse.z, 0)),
        (q = -q - (this.m_impulse.z - k.z) * this.m_K.col3.x),
        (a = -a - (this.m_impulse.z - k.z) * this.m_K.col3.y),
        (a = this.m_K.Solve22(new h(), q, a)),
        (a.x += k.x),
        (a.y += k.y),
        (this.m_impulse.x = a.x),
        (this.m_impulse.y = a.y),
        (b.x = this.m_impulse.x - k.x),
        (b.y = this.m_impulse.y - k.y),
        (b.z = this.m_impulse.z - k.z),
        (k = b.x * this.m_perp.x + b.z * this.m_axis.x),
        (a = b.x * this.m_perp.y + b.z * this.m_axis.y),
        (q = b.x * this.m_s1 + b.y + b.z * this.m_a1),
        (b = b.x * this.m_s2 + b.y + b.z * this.m_a2))
      : ((b = this.m_K.Solve22(new h(), -q, -a)),
        (this.m_impulse.x += b.x),
        (this.m_impulse.y += b.y),
        (k = b.x * this.m_perp.x),
        (a = b.x * this.m_perp.y),
        (q = b.x * this.m_s1 + b.y),
        (b = b.x * this.m_s2 + b.y));
    f.x -= this.m_invMassA * k;
    f.y -= this.m_invMassA * a;
    g -= this.m_invIA * q;
    v.x += this.m_invMassB * k;
    v.y += this.m_invMassB * a;
    n += this.m_invIB * b;
    d.m_linearVelocity.SetV(f);
    d.m_angularVelocity = g;
    e.m_linearVelocity.SetV(v);
    e.m_angularVelocity = n;
  };
  K.prototype.SolvePositionConstraints = function (a) {
    a = this.m_bodyA;
    var d = this.m_bodyB,
      e = a.m_sweep.c,
      f = a.m_sweep.a,
      g = d.m_sweep.c,
      v = d.m_sweep.a,
      t = 0;
    var b = !1;
    var k = 0,
      q = x.FromAngle(f),
      m = x.FromAngle(v);
    var p = q;
    var u = this.m_localAnchor1.x - this.m_localCenterA.x;
    var w = this.m_localAnchor1.y - this.m_localCenterA.y;
    var y = p.col1.x * u + p.col2.x * w;
    w = p.col1.y * u + p.col2.y * w;
    u = y;
    p = m;
    m = this.m_localAnchor2.x - this.m_localCenterB.x;
    var A = this.m_localAnchor2.y - this.m_localCenterB.y;
    y = p.col1.x * m + p.col2.x * A;
    A = p.col1.y * m + p.col2.y * A;
    m = y;
    p = g.x + m - e.x - u;
    y = g.y + A - e.y - w;
    if (this.m_enableLimit) {
      this.m_axis = c.MulMV(q, this.m_localXAxis1);
      this.m_a1 = (p + u) * this.m_axis.y - (y + w) * this.m_axis.x;
      this.m_a2 = m * this.m_axis.y - A * this.m_axis.x;
      var C = this.m_axis.x * p + this.m_axis.y * y;
      c.Abs(this.m_upperTranslation - this.m_lowerTranslation) <
      2 * n.b2_linearSlop
        ? ((k = c.Clamp(
            C,
            -n.b2_maxLinearCorrection,
            n.b2_maxLinearCorrection
          )),
          (t = c.Abs(C)),
          (b = !0))
        : C <= this.m_lowerTranslation
        ? ((k = c.Clamp(
            C - this.m_lowerTranslation + n.b2_linearSlop,
            -n.b2_maxLinearCorrection,
            0
          )),
          (t = this.m_lowerTranslation - C),
          (b = !0))
        : C >= this.m_upperTranslation &&
          ((k = c.Clamp(
            C - this.m_upperTranslation + n.b2_linearSlop,
            0,
            n.b2_maxLinearCorrection
          )),
          (t = C - this.m_upperTranslation),
          (b = !0));
    }
    this.m_perp = c.MulMV(q, this.m_localYAxis1);
    this.m_s1 = (p + u) * this.m_perp.y - (y + w) * this.m_perp.x;
    this.m_s2 = m * this.m_perp.y - A * this.m_perp.x;
    q = new l();
    w = this.m_perp.x * p + this.m_perp.y * y;
    m = v - f - this.m_refAngle;
    t = c.Max(t, c.Abs(w));
    u = c.Abs(m);
    b
      ? ((b = this.m_invMassA),
        (A = this.m_invMassB),
        (p = this.m_invIA),
        (y = this.m_invIB),
        (this.m_K.col1.x =
          b + A + p * this.m_s1 * this.m_s1 + y * this.m_s2 * this.m_s2),
        (this.m_K.col1.y = p * this.m_s1 + y * this.m_s2),
        (this.m_K.col1.z =
          p * this.m_s1 * this.m_a1 + y * this.m_s2 * this.m_a2),
        (this.m_K.col2.x = this.m_K.col1.y),
        (this.m_K.col2.y = p + y),
        (this.m_K.col2.z = p * this.m_a1 + y * this.m_a2),
        (this.m_K.col3.x = this.m_K.col1.z),
        (this.m_K.col3.y = this.m_K.col2.z),
        (this.m_K.col3.z =
          b + A + p * this.m_a1 * this.m_a1 + y * this.m_a2 * this.m_a2),
        this.m_K.Solve33(q, -w, -m, -k))
      : ((b = this.m_invMassA),
        (A = this.m_invMassB),
        (p = this.m_invIA),
        (y = this.m_invIB),
        (k = p * this.m_s1 + y * this.m_s2),
        (C = p + y),
        this.m_K.col1.Set(
          b + A + p * this.m_s1 * this.m_s1 + y * this.m_s2 * this.m_s2,
          k,
          0
        ),
        this.m_K.col2.Set(k, C, 0),
        (k = this.m_K.Solve22(new h(), -w, -m)),
        (q.x = k.x),
        (q.y = k.y),
        (q.z = 0));
    k = q.x * this.m_perp.x + q.z * this.m_axis.x;
    b = q.x * this.m_perp.y + q.z * this.m_axis.y;
    w = q.x * this.m_s1 + q.y + q.z * this.m_a1;
    q = q.x * this.m_s2 + q.y + q.z * this.m_a2;
    e.x -= this.m_invMassA * k;
    e.y -= this.m_invMassA * b;
    f -= this.m_invIA * w;
    g.x += this.m_invMassB * k;
    g.y += this.m_invMassB * b;
    v += this.m_invIB * q;
    a.m_sweep.a = f;
    d.m_sweep.a = v;
    a.SynchronizeTransform();
    d.SynchronizeTransform();
    return t <= n.b2_linearSlop && u <= n.b2_angularSlop;
  };
  Box2D.inherit(N, Box2D.Dynamics.Joints.b2JointDef);
  N.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
  N.b2PrismaticJointDef = function () {
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
    this.localAnchorA = new h();
    this.localAnchorB = new h();
    this.localAxisA = new h();
  };
  N.prototype.b2PrismaticJointDef = function () {
    this.__super.b2JointDef.call(this);
    this.type = y.e_prismaticJoint;
    this.localAxisA.Set(1, 0);
    this.referenceAngle = 0;
    this.enableLimit = !1;
    this.upperTranslation = this.lowerTranslation = 0;
    this.enableMotor = !1;
    this.motorSpeed = this.maxMotorForce = 0;
  };
  N.prototype.Initialize = function (a, d, c, f) {
    this.bodyA = a;
    this.bodyB = d;
    this.localAnchorA = this.bodyA.GetLocalPoint(c);
    this.localAnchorB = this.bodyB.GetLocalPoint(c);
    this.localAxisA = this.bodyA.GetLocalVector(f);
    this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
  };
  Box2D.inherit(D, Box2D.Dynamics.Joints.b2Joint);
  D.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
  D.b2PulleyJoint = function () {
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
    this.m_groundAnchor1 = new h();
    this.m_groundAnchor2 = new h();
    this.m_localAnchor1 = new h();
    this.m_localAnchor2 = new h();
    this.m_u1 = new h();
    this.m_u2 = new h();
  };
  D.prototype.GetAnchorA = function () {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
  };
  D.prototype.GetAnchorB = function () {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
  };
  D.prototype.GetReactionForce = function (a) {
    void 0 === a && (a = 0);
    return new h(
      a * this.m_impulse * this.m_u2.x,
      a * this.m_impulse * this.m_u2.y
    );
  };
  D.prototype.GetReactionTorque = function (a) {
    return 0;
  };
  D.prototype.GetGroundAnchorA = function () {
    var a = this.m_ground.m_xf.position.Copy();
    a.Add(this.m_groundAnchor1);
    return a;
  };
  D.prototype.GetGroundAnchorB = function () {
    var a = this.m_ground.m_xf.position.Copy();
    a.Add(this.m_groundAnchor2);
    return a;
  };
  D.prototype.GetLength1 = function () {
    var a = this.m_bodyA.GetWorldPoint(this.m_localAnchor1),
      d = a.x - (this.m_ground.m_xf.position.x + this.m_groundAnchor1.x),
      a = a.y - (this.m_ground.m_xf.position.y + this.m_groundAnchor1.y);
    return Math.sqrt(d * d + a * a);
  };
  D.prototype.GetLength2 = function () {
    var a = this.m_bodyB.GetWorldPoint(this.m_localAnchor2),
      d = a.x - (this.m_ground.m_xf.position.x + this.m_groundAnchor2.x),
      a = a.y - (this.m_ground.m_xf.position.y + this.m_groundAnchor2.y);
    return Math.sqrt(d * d + a * a);
  };
  D.prototype.GetRatio = function () {
    return this.m_ratio;
  };
  D.prototype.b2PulleyJoint = function (a) {
    this.__super.b2Joint.call(this, a);
    this.m_ground = this.m_bodyA.m_world.m_groundBody;
    this.m_groundAnchor1.x = a.groundAnchorA.x - this.m_ground.m_xf.position.x;
    this.m_groundAnchor1.y = a.groundAnchorA.y - this.m_ground.m_xf.position.y;
    this.m_groundAnchor2.x = a.groundAnchorB.x - this.m_ground.m_xf.position.x;
    this.m_groundAnchor2.y = a.groundAnchorB.y - this.m_ground.m_xf.position.y;
    this.m_localAnchor1.SetV(a.localAnchorA);
    this.m_localAnchor2.SetV(a.localAnchorB);
    this.m_ratio = a.ratio;
    this.m_constant = a.lengthA + this.m_ratio * a.lengthB;
    this.m_maxLength1 = c.Min(
      a.maxLengthA,
      this.m_constant - this.m_ratio * D.b2_minPulleyLength
    );
    this.m_maxLength2 = c.Min(
      a.maxLengthB,
      (this.m_constant - D.b2_minPulleyLength) / this.m_ratio
    );
    this.m_limitImpulse2 = this.m_limitImpulse1 = this.m_impulse = 0;
  };
  D.prototype.InitVelocityConstraints = function (a) {
    var d = this.m_bodyA,
      c = this.m_bodyB;
    var f = d.m_xf.R;
    var e = this.m_localAnchor1.x - d.m_sweep.localCenter.x,
      g = this.m_localAnchor1.y - d.m_sweep.localCenter.y,
      h = f.col1.x * e + f.col2.x * g,
      g = f.col1.y * e + f.col2.y * g,
      e = h;
    f = c.m_xf.R;
    var b = this.m_localAnchor2.x - c.m_sweep.localCenter.x,
      k = this.m_localAnchor2.y - c.m_sweep.localCenter.y,
      h = f.col1.x * b + f.col2.x * k,
      k = f.col1.y * b + f.col2.y * k,
      b = h;
    f = c.m_sweep.c.x + b;
    var h = c.m_sweep.c.y + k,
      l = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x,
      m = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
    this.m_u1.Set(
      d.m_sweep.c.x +
        e -
        (this.m_ground.m_xf.position.x + this.m_groundAnchor1.x),
      d.m_sweep.c.y +
        g -
        (this.m_ground.m_xf.position.y + this.m_groundAnchor1.y)
    );
    this.m_u2.Set(f - l, h - m);
    f = this.m_u1.Length();
    h = this.m_u2.Length();
    f > n.b2_linearSlop ? this.m_u1.Multiply(1 / f) : this.m_u1.SetZero();
    h > n.b2_linearSlop ? this.m_u2.Multiply(1 / h) : this.m_u2.SetZero();
    0 < this.m_constant - f - this.m_ratio * h
      ? ((this.m_state = y.e_inactiveLimit), (this.m_impulse = 0))
      : (this.m_state = y.e_atUpperLimit);
    f < this.m_maxLength1
      ? ((this.m_limitState1 = y.e_inactiveLimit), (this.m_limitImpulse1 = 0))
      : (this.m_limitState1 = y.e_atUpperLimit);
    h < this.m_maxLength2
      ? ((this.m_limitState2 = y.e_inactiveLimit), (this.m_limitImpulse2 = 0))
      : (this.m_limitState2 = y.e_atUpperLimit);
    f = e * this.m_u1.y - g * this.m_u1.x;
    h = b * this.m_u2.y - k * this.m_u2.x;
    this.m_limitMass1 = d.m_invMass + d.m_invI * f * f;
    this.m_limitMass2 = c.m_invMass + c.m_invI * h * h;
    this.m_pulleyMass =
      this.m_limitMass1 + this.m_ratio * this.m_ratio * this.m_limitMass2;
    this.m_limitMass1 = 1 / this.m_limitMass1;
    this.m_limitMass2 = 1 / this.m_limitMass2;
    this.m_pulleyMass = 1 / this.m_pulleyMass;
    a.warmStarting
      ? ((this.m_impulse *= a.dtRatio),
        (this.m_limitImpulse1 *= a.dtRatio),
        (this.m_limitImpulse2 *= a.dtRatio),
        (a = (-this.m_impulse - this.m_limitImpulse1) * this.m_u1.x),
        (f = (-this.m_impulse - this.m_limitImpulse1) * this.m_u1.y),
        (h =
          (-this.m_ratio * this.m_impulse - this.m_limitImpulse2) *
          this.m_u2.x),
        (l =
          (-this.m_ratio * this.m_impulse - this.m_limitImpulse2) *
          this.m_u2.y),
        (d.m_linearVelocity.x += d.m_invMass * a),
        (d.m_linearVelocity.y += d.m_invMass * f),
        (d.m_angularVelocity += d.m_invI * (e * f - g * a)),
        (c.m_linearVelocity.x += c.m_invMass * h),
        (c.m_linearVelocity.y += c.m_invMass * l),
        (c.m_angularVelocity += c.m_invI * (b * l - k * h)))
      : (this.m_limitImpulse2 = this.m_limitImpulse1 = this.m_impulse = 0);
  };
  D.prototype.SolveVelocityConstraints = function (a) {
    a = this.m_bodyA;
    var d = this.m_bodyB;
    var e = a.m_xf.R;
    var f = this.m_localAnchor1.x - a.m_sweep.localCenter.x,
      g = this.m_localAnchor1.y - a.m_sweep.localCenter.y,
      h = e.col1.x * f + e.col2.x * g,
      g = e.col1.y * f + e.col2.y * g,
      f = h;
    e = d.m_xf.R;
    var l = this.m_localAnchor2.x - d.m_sweep.localCenter.x,
      b = this.m_localAnchor2.y - d.m_sweep.localCenter.y,
      h = e.col1.x * l + e.col2.x * b,
      b = e.col1.y * l + e.col2.y * b,
      l = h;
    if (this.m_state == y.e_atUpperLimit) {
      e = a.m_linearVelocity.x + -a.m_angularVelocity * g;
      h = a.m_linearVelocity.y + a.m_angularVelocity * f;
      var k = d.m_linearVelocity.x + -d.m_angularVelocity * b;
      var q = d.m_linearVelocity.y + d.m_angularVelocity * l;
      e =
        -(this.m_u1.x * e + this.m_u1.y * h) -
        this.m_ratio * (this.m_u2.x * k + this.m_u2.y * q);
      q = this.m_pulleyMass * -e;
      e = this.m_impulse;
      this.m_impulse = c.Max(0, this.m_impulse + q);
      q = this.m_impulse - e;
      e = -q * this.m_u1.x;
      h = -q * this.m_u1.y;
      k = -this.m_ratio * q * this.m_u2.x;
      q = -this.m_ratio * q * this.m_u2.y;
      a.m_linearVelocity.x += a.m_invMass * e;
      a.m_linearVelocity.y += a.m_invMass * h;
      a.m_angularVelocity += a.m_invI * (f * h - g * e);
      d.m_linearVelocity.x += d.m_invMass * k;
      d.m_linearVelocity.y += d.m_invMass * q;
      d.m_angularVelocity += d.m_invI * (l * q - b * k);
    }
    this.m_limitState1 == y.e_atUpperLimit &&
      ((e = a.m_linearVelocity.x + -a.m_angularVelocity * g),
      (h = a.m_linearVelocity.y + a.m_angularVelocity * f),
      (e = -(this.m_u1.x * e + this.m_u1.y * h)),
      (q = -this.m_limitMass1 * e),
      (e = this.m_limitImpulse1),
      (this.m_limitImpulse1 = c.Max(0, this.m_limitImpulse1 + q)),
      (q = this.m_limitImpulse1 - e),
      (e = -q * this.m_u1.x),
      (h = -q * this.m_u1.y),
      (a.m_linearVelocity.x += a.m_invMass * e),
      (a.m_linearVelocity.y += a.m_invMass * h),
      (a.m_angularVelocity += a.m_invI * (f * h - g * e)));
    this.m_limitState2 == y.e_atUpperLimit &&
      ((k = d.m_linearVelocity.x + -d.m_angularVelocity * b),
      (q = d.m_linearVelocity.y + d.m_angularVelocity * l),
      (e = -(this.m_u2.x * k + this.m_u2.y * q)),
      (q = -this.m_limitMass2 * e),
      (e = this.m_limitImpulse2),
      (this.m_limitImpulse2 = c.Max(0, this.m_limitImpulse2 + q)),
      (q = this.m_limitImpulse2 - e),
      (k = -q * this.m_u2.x),
      (q = -q * this.m_u2.y),
      (d.m_linearVelocity.x += d.m_invMass * k),
      (d.m_linearVelocity.y += d.m_invMass * q),
      (d.m_angularVelocity += d.m_invI * (l * q - b * k)));
  };
  D.prototype.SolvePositionConstraints = function (a) {
    a = this.m_bodyA;
    var d = this.m_bodyB,
      e = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x,
      f = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y,
      g = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x,
      h = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y,
      l = 0;
    if (this.m_state == y.e_atUpperLimit) {
      var b = a.m_xf.R;
      var k = this.m_localAnchor1.x - a.m_sweep.localCenter.x;
      var q = this.m_localAnchor1.y - a.m_sweep.localCenter.y;
      var m = b.col1.x * k + b.col2.x * q;
      q = b.col1.y * k + b.col2.y * q;
      k = m;
      b = d.m_xf.R;
      var p = this.m_localAnchor2.x - d.m_sweep.localCenter.x;
      var x = this.m_localAnchor2.y - d.m_sweep.localCenter.y;
      m = b.col1.x * p + b.col2.x * x;
      x = b.col1.y * p + b.col2.y * x;
      p = m;
      b = a.m_sweep.c.x + k;
      m = a.m_sweep.c.y + q;
      var t = d.m_sweep.c.x + p;
      var u = d.m_sweep.c.y + x;
      this.m_u1.Set(b - e, m - f);
      this.m_u2.Set(t - g, u - h);
      b = this.m_u1.Length();
      m = this.m_u2.Length();
      b > n.b2_linearSlop ? this.m_u1.Multiply(1 / b) : this.m_u1.SetZero();
      m > n.b2_linearSlop ? this.m_u2.Multiply(1 / m) : this.m_u2.SetZero();
      b = this.m_constant - b - this.m_ratio * m;
      l = c.Max(l, -b);
      b = c.Clamp(b + n.b2_linearSlop, -n.b2_maxLinearCorrection, 0);
      u = -this.m_pulleyMass * b;
      b = -u * this.m_u1.x;
      m = -u * this.m_u1.y;
      t = -this.m_ratio * u * this.m_u2.x;
      u = -this.m_ratio * u * this.m_u2.y;
      a.m_sweep.c.x += a.m_invMass * b;
      a.m_sweep.c.y += a.m_invMass * m;
      a.m_sweep.a += a.m_invI * (k * m - q * b);
      d.m_sweep.c.x += d.m_invMass * t;
      d.m_sweep.c.y += d.m_invMass * u;
      d.m_sweep.a += d.m_invI * (p * u - x * t);
      a.SynchronizeTransform();
      d.SynchronizeTransform();
    }
    this.m_limitState1 == y.e_atUpperLimit &&
      ((b = a.m_xf.R),
      (k = this.m_localAnchor1.x - a.m_sweep.localCenter.x),
      (q = this.m_localAnchor1.y - a.m_sweep.localCenter.y),
      (m = b.col1.x * k + b.col2.x * q),
      (q = b.col1.y * k + b.col2.y * q),
      (k = m),
      (b = a.m_sweep.c.x + k),
      (m = a.m_sweep.c.y + q),
      this.m_u1.Set(b - e, m - f),
      (b = this.m_u1.Length()),
      b > n.b2_linearSlop
        ? ((this.m_u1.x *= 1 / b), (this.m_u1.y *= 1 / b))
        : this.m_u1.SetZero(),
      (b = this.m_maxLength1 - b),
      (l = c.Max(l, -b)),
      (b = c.Clamp(b + n.b2_linearSlop, -n.b2_maxLinearCorrection, 0)),
      (u = -this.m_limitMass1 * b),
      (b = -u * this.m_u1.x),
      (m = -u * this.m_u1.y),
      (a.m_sweep.c.x += a.m_invMass * b),
      (a.m_sweep.c.y += a.m_invMass * m),
      (a.m_sweep.a += a.m_invI * (k * m - q * b)),
      a.SynchronizeTransform());
    this.m_limitState2 == y.e_atUpperLimit &&
      ((b = d.m_xf.R),
      (p = this.m_localAnchor2.x - d.m_sweep.localCenter.x),
      (x = this.m_localAnchor2.y - d.m_sweep.localCenter.y),
      (m = b.col1.x * p + b.col2.x * x),
      (x = b.col1.y * p + b.col2.y * x),
      (p = m),
      (t = d.m_sweep.c.x + p),
      (u = d.m_sweep.c.y + x),
      this.m_u2.Set(t - g, u - h),
      (m = this.m_u2.Length()),
      m > n.b2_linearSlop
        ? ((this.m_u2.x *= 1 / m), (this.m_u2.y *= 1 / m))
        : this.m_u2.SetZero(),
      (b = this.m_maxLength2 - m),
      (l = c.Max(l, -b)),
      (b = c.Clamp(b + n.b2_linearSlop, -n.b2_maxLinearCorrection, 0)),
      (u = -this.m_limitMass2 * b),
      (t = -u * this.m_u2.x),
      (u = -u * this.m_u2.y),
      (d.m_sweep.c.x += d.m_invMass * t),
      (d.m_sweep.c.y += d.m_invMass * u),
      (d.m_sweep.a += d.m_invI * (p * u - x * t)),
      d.SynchronizeTransform());
    return l < n.b2_linearSlop;
  };
  Box2D.postDefs.push(function () {
    Box2D.Dynamics.Joints.b2PulleyJoint.b2_minPulleyLength = 2;
  });
  Box2D.inherit(R, Box2D.Dynamics.Joints.b2JointDef);
  R.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
  R.b2PulleyJointDef = function () {
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
    this.groundAnchorA = new h();
    this.groundAnchorB = new h();
    this.localAnchorA = new h();
    this.localAnchorB = new h();
  };
  R.prototype.b2PulleyJointDef = function () {
    this.__super.b2JointDef.call(this);
    this.type = y.e_pulleyJoint;
    this.groundAnchorA.Set(-1, 1);
    this.groundAnchorB.Set(1, 1);
    this.localAnchorA.Set(-1, 0);
    this.localAnchorB.Set(1, 0);
    this.maxLengthB = this.lengthB = this.maxLengthA = this.lengthA = 0;
    this.ratio = 1;
    this.collideConnected = !0;
  };
  R.prototype.Initialize = function (a, c, e, f, g, h, l) {
    void 0 === l && (l = 0);
    this.bodyA = a;
    this.bodyB = c;
    this.groundAnchorA.SetV(e);
    this.groundAnchorB.SetV(f);
    this.localAnchorA = this.bodyA.GetLocalPoint(g);
    this.localAnchorB = this.bodyB.GetLocalPoint(h);
    a = g.x - e.x;
    e = g.y - e.y;
    this.lengthA = Math.sqrt(a * a + e * e);
    e = h.x - f.x;
    f = h.y - f.y;
    this.lengthB = Math.sqrt(e * e + f * f);
    this.ratio = l;
    l = this.lengthA + this.ratio * this.lengthB;
    this.maxLengthA = l - this.ratio * D.b2_minPulleyLength;
    this.maxLengthB = (l - D.b2_minPulleyLength) / this.ratio;
  };
  Box2D.inherit(F, Box2D.Dynamics.Joints.b2Joint);
  F.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
  F.b2RevoluteJoint = function () {
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
    this.K = new x();
    this.K1 = new x();
    this.K2 = new x();
    this.K3 = new x();
    this.impulse3 = new l();
    this.impulse2 = new h();
    this.reduced = new h();
    this.m_localAnchor1 = new h();
    this.m_localAnchor2 = new h();
    this.m_impulse = new l();
    this.m_mass = new t();
  };
  F.prototype.GetAnchorA = function () {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
  };
  F.prototype.GetAnchorB = function () {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
  };
  F.prototype.GetReactionForce = function (a) {
    void 0 === a && (a = 0);
    return new h(a * this.m_impulse.x, a * this.m_impulse.y);
  };
  F.prototype.GetReactionTorque = function (a) {
    void 0 === a && (a = 0);
    return a * this.m_impulse.z;
  };
  F.prototype.GetJointAngle = function () {
    return (
      this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a - this.m_referenceAngle
    );
  };
  F.prototype.GetJointSpeed = function () {
    return this.m_bodyB.m_angularVelocity - this.m_bodyA.m_angularVelocity;
  };
  F.prototype.IsLimitEnabled = function () {
    return this.m_enableLimit;
  };
  F.prototype.EnableLimit = function (a) {
    this.m_enableLimit = a;
  };
  F.prototype.GetLowerLimit = function () {
    return this.m_lowerAngle;
  };
  F.prototype.GetUpperLimit = function () {
    return this.m_upperAngle;
  };
  F.prototype.SetLimits = function (a, c) {
    void 0 === a && (a = 0);
    void 0 === c && (c = 0);
    this.m_lowerAngle = a;
    this.m_upperAngle = c;
  };
  F.prototype.IsMotorEnabled = function () {
    this.m_bodyA.SetAwake(!0);
    this.m_bodyB.SetAwake(!0);
    return this.m_enableMotor;
  };
  F.prototype.EnableMotor = function (a) {
    this.m_enableMotor = a;
  };
  F.prototype.SetMotorSpeed = function (a) {
    void 0 === a && (a = 0);
    this.m_bodyA.SetAwake(!0);
    this.m_bodyB.SetAwake(!0);
    this.m_motorSpeed = a;
  };
  F.prototype.GetMotorSpeed = function () {
    return this.m_motorSpeed;
  };
  F.prototype.SetMaxMotorTorque = function (a) {
    void 0 === a && (a = 0);
    this.m_maxMotorTorque = a;
  };
  F.prototype.GetMotorTorque = function () {
    return this.m_maxMotorTorque;
  };
  F.prototype.b2RevoluteJoint = function (a) {
    this.__super.b2Joint.call(this, a);
    this.m_localAnchor1.SetV(a.localAnchorA);
    this.m_localAnchor2.SetV(a.localAnchorB);
    this.m_referenceAngle = a.referenceAngle;
    this.m_impulse.SetZero();
    this.m_motorImpulse = 0;
    this.m_lowerAngle = a.lowerAngle;
    this.m_upperAngle = a.upperAngle;
    this.m_maxMotorTorque = a.maxMotorTorque;
    this.m_motorSpeed = a.motorSpeed;
    this.m_enableLimit = a.enableLimit;
    this.m_enableMotor = a.enableMotor;
    this.m_limitState = y.e_inactiveLimit;
  };
  F.prototype.InitVelocityConstraints = function (a) {
    var d = this.m_bodyA,
      e = this.m_bodyB;
    var f = d.m_xf.R;
    var g = this.m_localAnchor1.x - d.m_sweep.localCenter.x,
      h = this.m_localAnchor1.y - d.m_sweep.localCenter.y;
    var l = f.col1.x * g + f.col2.x * h;
    h = f.col1.y * g + f.col2.y * h;
    g = l;
    f = e.m_xf.R;
    var b = this.m_localAnchor2.x - e.m_sweep.localCenter.x,
      k = this.m_localAnchor2.y - e.m_sweep.localCenter.y;
    l = f.col1.x * b + f.col2.x * k;
    k = f.col1.y * b + f.col2.y * k;
    b = l;
    f = d.m_invMass;
    l = e.m_invMass;
    var q = d.m_invI,
      m = e.m_invI;
    this.m_mass.col1.x = f + l + h * h * q + k * k * m;
    this.m_mass.col2.x = -h * g * q - k * b * m;
    this.m_mass.col3.x = -h * q - k * m;
    this.m_mass.col1.y = this.m_mass.col2.x;
    this.m_mass.col2.y = f + l + g * g * q + b * b * m;
    this.m_mass.col3.y = g * q + b * m;
    this.m_mass.col1.z = this.m_mass.col3.x;
    this.m_mass.col2.z = this.m_mass.col3.y;
    this.m_mass.col3.z = q + m;
    this.m_motorMass = 1 / (q + m);
    0 == this.m_enableMotor && (this.m_motorImpulse = 0);
    if (this.m_enableLimit) {
      var p = e.m_sweep.a - d.m_sweep.a - this.m_referenceAngle;
      c.Abs(this.m_upperAngle - this.m_lowerAngle) < 2 * n.b2_angularSlop
        ? (this.m_limitState = y.e_equalLimits)
        : p <= this.m_lowerAngle
        ? (this.m_limitState != y.e_atLowerLimit && (this.m_impulse.z = 0),
          (this.m_limitState = y.e_atLowerLimit))
        : p >= this.m_upperAngle
        ? (this.m_limitState != y.e_atUpperLimit && (this.m_impulse.z = 0),
          (this.m_limitState = y.e_atUpperLimit))
        : ((this.m_limitState = y.e_inactiveLimit), (this.m_impulse.z = 0));
    } else this.m_limitState = y.e_inactiveLimit;
    a.warmStarting
      ? ((this.m_impulse.x *= a.dtRatio),
        (this.m_impulse.y *= a.dtRatio),
        (this.m_motorImpulse *= a.dtRatio),
        (a = this.m_impulse.x),
        (p = this.m_impulse.y),
        (d.m_linearVelocity.x -= f * a),
        (d.m_linearVelocity.y -= f * p),
        (d.m_angularVelocity -=
          q * (g * p - h * a + this.m_motorImpulse + this.m_impulse.z)),
        (e.m_linearVelocity.x += l * a),
        (e.m_linearVelocity.y += l * p),
        (e.m_angularVelocity +=
          m * (b * p - k * a + this.m_motorImpulse + this.m_impulse.z)))
      : (this.m_impulse.SetZero(), (this.m_motorImpulse = 0));
  };
  F.prototype.SolveVelocityConstraints = function (a) {
    var d = this.m_bodyA,
      e = this.m_bodyB,
      f = d.m_linearVelocity,
      g = d.m_angularVelocity,
      h = e.m_linearVelocity,
      l = e.m_angularVelocity,
      b = d.m_invMass,
      k = e.m_invMass,
      q = d.m_invI,
      m = e.m_invI;
    if (this.m_enableMotor && this.m_limitState != y.e_equalLimits) {
      var p = this.m_motorMass * -(l - g - this.m_motorSpeed);
      var n = this.m_motorImpulse;
      a = a.dt * this.m_maxMotorTorque;
      this.m_motorImpulse = c.Clamp(this.m_motorImpulse + p, -a, a);
      p = this.m_motorImpulse - n;
      g -= q * p;
      l += m * p;
    }
    if (this.m_enableLimit && this.m_limitState != y.e_inactiveLimit) {
      var x = d.m_xf.R;
      p = this.m_localAnchor1.x - d.m_sweep.localCenter.x;
      n = this.m_localAnchor1.y - d.m_sweep.localCenter.y;
      var u = x.col1.x * p + x.col2.x * n;
      n = x.col1.y * p + x.col2.y * n;
      p = u;
      x = e.m_xf.R;
      a = this.m_localAnchor2.x - e.m_sweep.localCenter.x;
      var t = this.m_localAnchor2.y - e.m_sweep.localCenter.y;
      u = x.col1.x * a + x.col2.x * t;
      t = x.col1.y * a + x.col2.y * t;
      a = u;
      u = h.x + -l * t - f.x - -g * n;
      var w = h.y + l * a - f.y - g * p;
      this.m_mass.Solve33(this.impulse3, -u, -w, -(l - g));
      this.m_limitState == y.e_equalLimits
        ? this.m_impulse.Add(this.impulse3)
        : this.m_limitState == y.e_atLowerLimit
        ? ((x = this.m_impulse.z + this.impulse3.z),
          0 > x &&
            (this.m_mass.Solve22(this.reduced, -u, -w),
            (this.impulse3.x = this.reduced.x),
            (this.impulse3.y = this.reduced.y),
            (this.impulse3.z = -this.m_impulse.z),
            (this.m_impulse.x += this.reduced.x),
            (this.m_impulse.y += this.reduced.y),
            (this.m_impulse.z = 0)))
        : this.m_limitState == y.e_atUpperLimit &&
          ((x = this.m_impulse.z + this.impulse3.z),
          0 < x &&
            (this.m_mass.Solve22(this.reduced, -u, -w),
            (this.impulse3.x = this.reduced.x),
            (this.impulse3.y = this.reduced.y),
            (this.impulse3.z = -this.m_impulse.z),
            (this.m_impulse.x += this.reduced.x),
            (this.m_impulse.y += this.reduced.y),
            (this.m_impulse.z = 0)));
      f.x -= b * this.impulse3.x;
      f.y -= b * this.impulse3.y;
      g -= q * (p * this.impulse3.y - n * this.impulse3.x + this.impulse3.z);
      h.x += k * this.impulse3.x;
      h.y += k * this.impulse3.y;
      l += m * (a * this.impulse3.y - t * this.impulse3.x + this.impulse3.z);
    } else
      (x = d.m_xf.R),
        (p = this.m_localAnchor1.x - d.m_sweep.localCenter.x),
        (n = this.m_localAnchor1.y - d.m_sweep.localCenter.y),
        (u = x.col1.x * p + x.col2.x * n),
        (n = x.col1.y * p + x.col2.y * n),
        (p = u),
        (x = e.m_xf.R),
        (a = this.m_localAnchor2.x - e.m_sweep.localCenter.x),
        (t = this.m_localAnchor2.y - e.m_sweep.localCenter.y),
        (u = x.col1.x * a + x.col2.x * t),
        (t = x.col1.y * a + x.col2.y * t),
        (a = u),
        this.m_mass.Solve22(
          this.impulse2,
          -(h.x + -l * t - f.x - -g * n),
          -(h.y + l * a - f.y - g * p)
        ),
        (this.m_impulse.x += this.impulse2.x),
        (this.m_impulse.y += this.impulse2.y),
        (f.x -= b * this.impulse2.x),
        (f.y -= b * this.impulse2.y),
        (g -= q * (p * this.impulse2.y - n * this.impulse2.x)),
        (h.x += k * this.impulse2.x),
        (h.y += k * this.impulse2.y),
        (l += m * (a * this.impulse2.y - t * this.impulse2.x));
    d.m_linearVelocity.SetV(f);
    d.m_angularVelocity = g;
    e.m_linearVelocity.SetV(h);
    e.m_angularVelocity = l;
  };
  F.prototype.SolvePositionConstraints = function (a) {
    a = this.m_bodyA;
    var d = this.m_bodyB,
      e = 0;
    if (this.m_enableLimit && this.m_limitState != y.e_inactiveLimit) {
      var f = d.m_sweep.a - a.m_sweep.a - this.m_referenceAngle;
      var g = 0;
      this.m_limitState == y.e_equalLimits
        ? ((f = c.Clamp(
            f - this.m_lowerAngle,
            -n.b2_maxAngularCorrection,
            n.b2_maxAngularCorrection
          )),
          (g = -this.m_motorMass * f),
          (e = c.Abs(f)))
        : this.m_limitState == y.e_atLowerLimit
        ? ((f -= this.m_lowerAngle),
          (e = -f),
          (f = c.Clamp(f + n.b2_angularSlop, -n.b2_maxAngularCorrection, 0)),
          (g = -this.m_motorMass * f))
        : this.m_limitState == y.e_atUpperLimit &&
          ((e = f -= this.m_upperAngle),
          (f = c.Clamp(f - n.b2_angularSlop, 0, n.b2_maxAngularCorrection)),
          (g = -this.m_motorMass * f));
      a.m_sweep.a -= a.m_invI * g;
      d.m_sweep.a += d.m_invI * g;
      a.SynchronizeTransform();
      d.SynchronizeTransform();
    }
    var h = a.m_xf.R;
    g = this.m_localAnchor1.x - a.m_sweep.localCenter.x;
    f = this.m_localAnchor1.y - a.m_sweep.localCenter.y;
    var l = h.col1.x * g + h.col2.x * f;
    f = h.col1.y * g + h.col2.y * f;
    g = l;
    h = d.m_xf.R;
    var b = this.m_localAnchor2.x - d.m_sweep.localCenter.x,
      k = this.m_localAnchor2.y - d.m_sweep.localCenter.y;
    l = h.col1.x * b + h.col2.x * k;
    k = h.col1.y * b + h.col2.y * k;
    b = l;
    var q = d.m_sweep.c.x + b - a.m_sweep.c.x - g;
    var m = d.m_sweep.c.y + k - a.m_sweep.c.y - f;
    var p = q * q + m * m;
    h = Math.sqrt(p);
    l = a.m_invMass;
    var x = d.m_invMass,
      u = a.m_invI,
      t = d.m_invI,
      w = 10 * n.b2_linearSlop;
    p > w * w &&
      ((p = 1 / (l + x)),
      (q = p * -q),
      (m = p * -m),
      (a.m_sweep.c.x -= 0.5 * l * q),
      (a.m_sweep.c.y -= 0.5 * l * m),
      (d.m_sweep.c.x += 0.5 * x * q),
      (d.m_sweep.c.y += 0.5 * x * m),
      (q = d.m_sweep.c.x + b - a.m_sweep.c.x - g),
      (m = d.m_sweep.c.y + k - a.m_sweep.c.y - f));
    this.K1.col1.x = l + x;
    this.K1.col2.x = 0;
    this.K1.col1.y = 0;
    this.K1.col2.y = l + x;
    this.K2.col1.x = u * f * f;
    this.K2.col2.x = -u * g * f;
    this.K2.col1.y = -u * g * f;
    this.K2.col2.y = u * g * g;
    this.K3.col1.x = t * k * k;
    this.K3.col2.x = -t * b * k;
    this.K3.col1.y = -t * b * k;
    this.K3.col2.y = t * b * b;
    this.K.SetM(this.K1);
    this.K.AddM(this.K2);
    this.K.AddM(this.K3);
    this.K.Solve(F.tImpulse, -q, -m);
    q = F.tImpulse.x;
    m = F.tImpulse.y;
    a.m_sweep.c.x -= a.m_invMass * q;
    a.m_sweep.c.y -= a.m_invMass * m;
    a.m_sweep.a -= a.m_invI * (g * m - f * q);
    d.m_sweep.c.x += d.m_invMass * q;
    d.m_sweep.c.y += d.m_invMass * m;
    d.m_sweep.a += d.m_invI * (b * m - k * q);
    a.SynchronizeTransform();
    d.SynchronizeTransform();
    return h <= n.b2_linearSlop && e <= n.b2_angularSlop;
  };
  Box2D.postDefs.push(function () {
    Box2D.Dynamics.Joints.b2RevoluteJoint.tImpulse = new h();
  });
  Box2D.inherit(I, Box2D.Dynamics.Joints.b2JointDef);
  I.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
  I.b2RevoluteJointDef = function () {
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
    this.localAnchorA = new h();
    this.localAnchorB = new h();
  };
  I.prototype.b2RevoluteJointDef = function () {
    this.__super.b2JointDef.call(this);
    this.type = y.e_revoluteJoint;
    this.localAnchorA.Set(0, 0);
    this.localAnchorB.Set(0, 0);
    this.motorSpeed =
      this.maxMotorTorque =
      this.upperAngle =
      this.lowerAngle =
      this.referenceAngle =
        0;
    this.enableMotor = this.enableLimit = !1;
  };
  I.prototype.Initialize = function (a, c, e) {
    this.bodyA = a;
    this.bodyB = c;
    this.localAnchorA = this.bodyA.GetLocalPoint(e);
    this.localAnchorB = this.bodyB.GetLocalPoint(e);
    this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
  };
  Box2D.inherit(S, Box2D.Dynamics.Joints.b2Joint);
  S.prototype.__super = Box2D.Dynamics.Joints.b2Joint.prototype;
  S.b2WeldJoint = function () {
    Box2D.Dynamics.Joints.b2Joint.b2Joint.apply(this, arguments);
    this.m_localAnchorA = new h();
    this.m_localAnchorB = new h();
    this.m_impulse = new l();
    this.m_mass = new t();
  };
  S.prototype.GetAnchorA = function () {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
  };
  S.prototype.GetAnchorB = function () {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
  };
  S.prototype.GetReactionForce = function (a) {
    void 0 === a && (a = 0);
    return new h(a * this.m_impulse.x, a * this.m_impulse.y);
  };
  S.prototype.GetReactionTorque = function (a) {
    void 0 === a && (a = 0);
    return a * this.m_impulse.z;
  };
  S.prototype.b2WeldJoint = function (a) {
    this.__super.b2Joint.call(this, a);
    this.m_localAnchorA.SetV(a.localAnchorA);
    this.m_localAnchorB.SetV(a.localAnchorB);
    this.m_referenceAngle = a.referenceAngle;
    this.m_impulse.SetZero();
    this.m_mass = new t();
  };
  S.prototype.InitVelocityConstraints = function (a) {
    var c = this.m_bodyA,
      e = this.m_bodyB;
    var f = c.m_xf.R;
    var g = this.m_localAnchorA.x - c.m_sweep.localCenter.x,
      h = this.m_localAnchorA.y - c.m_sweep.localCenter.y;
    var l = f.col1.x * g + f.col2.x * h;
    h = f.col1.y * g + f.col2.y * h;
    g = l;
    f = e.m_xf.R;
    var b = this.m_localAnchorB.x - e.m_sweep.localCenter.x,
      k = this.m_localAnchorB.y - e.m_sweep.localCenter.y;
    l = f.col1.x * b + f.col2.x * k;
    k = f.col1.y * b + f.col2.y * k;
    b = l;
    f = c.m_invMass;
    l = e.m_invMass;
    var q = c.m_invI,
      m = e.m_invI;
    this.m_mass.col1.x = f + l + h * h * q + k * k * m;
    this.m_mass.col2.x = -h * g * q - k * b * m;
    this.m_mass.col3.x = -h * q - k * m;
    this.m_mass.col1.y = this.m_mass.col2.x;
    this.m_mass.col2.y = f + l + g * g * q + b * b * m;
    this.m_mass.col3.y = g * q + b * m;
    this.m_mass.col1.z = this.m_mass.col3.x;
    this.m_mass.col2.z = this.m_mass.col3.y;
    this.m_mass.col3.z = q + m;
    a.warmStarting
      ? ((this.m_impulse.x *= a.dtRatio),
        (this.m_impulse.y *= a.dtRatio),
        (this.m_impulse.z *= a.dtRatio),
        (c.m_linearVelocity.x -= f * this.m_impulse.x),
        (c.m_linearVelocity.y -= f * this.m_impulse.y),
        (c.m_angularVelocity -=
          q * (g * this.m_impulse.y - h * this.m_impulse.x + this.m_impulse.z)),
        (e.m_linearVelocity.x += l * this.m_impulse.x),
        (e.m_linearVelocity.y += l * this.m_impulse.y),
        (e.m_angularVelocity +=
          m * (b * this.m_impulse.y - k * this.m_impulse.x + this.m_impulse.z)))
      : this.m_impulse.SetZero();
  };
  S.prototype.SolveVelocityConstraints = function (a) {
    a = this.m_bodyA;
    var c = this.m_bodyB,
      e = a.m_linearVelocity,
      f = a.m_angularVelocity,
      g = c.m_linearVelocity,
      h = c.m_angularVelocity,
      n = a.m_invMass,
      b = c.m_invMass,
      k = a.m_invI,
      q = c.m_invI;
    var m = a.m_xf.R;
    var p = this.m_localAnchorA.x - a.m_sweep.localCenter.x,
      x = this.m_localAnchorA.y - a.m_sweep.localCenter.y;
    var u = m.col1.x * p + m.col2.x * x;
    x = m.col1.y * p + m.col2.y * x;
    p = u;
    m = c.m_xf.R;
    var t = this.m_localAnchorB.x - c.m_sweep.localCenter.x,
      w = this.m_localAnchorB.y - c.m_sweep.localCenter.y;
    u = m.col1.x * t + m.col2.x * w;
    w = m.col1.y * t + m.col2.y * w;
    t = u;
    m = g.x - h * w - e.x + f * x;
    u = g.y + h * t - e.y - f * p;
    var y = h - f,
      A = new l();
    this.m_mass.Solve33(A, -m, -u, -y);
    this.m_impulse.Add(A);
    e.x -= n * A.x;
    e.y -= n * A.y;
    f -= k * (p * A.y - x * A.x + A.z);
    g.x += b * A.x;
    g.y += b * A.y;
    h += q * (t * A.y - w * A.x + A.z);
    a.m_angularVelocity = f;
    c.m_angularVelocity = h;
  };
  S.prototype.SolvePositionConstraints = function (a) {
    a = this.m_bodyA;
    var d = this.m_bodyB;
    var e = a.m_xf.R;
    var f = this.m_localAnchorA.x - a.m_sweep.localCenter.x,
      g = this.m_localAnchorA.y - a.m_sweep.localCenter.y;
    var h = e.col1.x * f + e.col2.x * g;
    g = e.col1.y * f + e.col2.y * g;
    f = h;
    e = d.m_xf.R;
    var x = this.m_localAnchorB.x - d.m_sweep.localCenter.x,
      b = this.m_localAnchorB.y - d.m_sweep.localCenter.y;
    h = e.col1.x * x + e.col2.x * b;
    b = e.col1.y * x + e.col2.y * b;
    x = h;
    e = a.m_invMass;
    h = d.m_invMass;
    var k = a.m_invI,
      q = d.m_invI,
      m = d.m_sweep.c.x + x - a.m_sweep.c.x - f,
      p = d.m_sweep.c.y + b - a.m_sweep.c.y - g,
      u = d.m_sweep.a - a.m_sweep.a - this.m_referenceAngle,
      t = 10 * n.b2_linearSlop,
      w = Math.sqrt(m * m + p * p),
      y = c.Abs(u);
    w > t && ((k *= 1), (q *= 1));
    this.m_mass.col1.x = e + h + g * g * k + b * b * q;
    this.m_mass.col2.x = -g * f * k - b * x * q;
    this.m_mass.col3.x = -g * k - b * q;
    this.m_mass.col1.y = this.m_mass.col2.x;
    this.m_mass.col2.y = e + h + f * f * k + x * x * q;
    this.m_mass.col3.y = f * k + x * q;
    this.m_mass.col1.z = this.m_mass.col3.x;
    this.m_mass.col2.z = this.m_mass.col3.y;
    this.m_mass.col3.z = k + q;
    t = new l();
    this.m_mass.Solve33(t, -m, -p, -u);
    a.m_sweep.c.x -= e * t.x;
    a.m_sweep.c.y -= e * t.y;
    a.m_sweep.a -= k * (f * t.y - g * t.x + t.z);
    d.m_sweep.c.x += h * t.x;
    d.m_sweep.c.y += h * t.y;
    d.m_sweep.a += q * (x * t.y - b * t.x + t.z);
    a.SynchronizeTransform();
    d.SynchronizeTransform();
    return w <= n.b2_linearSlop && y <= n.b2_angularSlop;
  };
  Box2D.inherit(O, Box2D.Dynamics.Joints.b2JointDef);
  O.prototype.__super = Box2D.Dynamics.Joints.b2JointDef.prototype;
  O.b2WeldJointDef = function () {
    Box2D.Dynamics.Joints.b2JointDef.b2JointDef.apply(this, arguments);
    this.localAnchorA = new h();
    this.localAnchorB = new h();
  };
  O.prototype.b2WeldJointDef = function () {
    this.__super.b2JointDef.call(this);
    this.type = y.e_weldJoint;
    this.referenceAngle = 0;
  };
  O.prototype.Initialize = function (a, c, e) {
    this.bodyA = a;
    this.bodyB = c;
    this.localAnchorA.SetV(this.bodyA.GetLocalPoint(e));
    this.localAnchorB.SetV(this.bodyB.GetLocalPoint(e));
    this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
  };
})();
(function () {
  var n = Box2D.Dynamics.b2DebugDraw;
  n.b2DebugDraw = function () {
    this.m_xformScale =
      this.m_fillAlpha =
      this.m_alpha =
      this.m_lineThickness =
      this.m_drawScale =
        1;
    var n = this;
    this.m_sprite = {
      graphics: {
        clear: function () {
          n.m_ctx.clearRect(0, 0, n.m_ctx.canvas.width, n.m_ctx.canvas.height);
        },
      },
    };
  };
  n.prototype._color = function (n, t) {
    return (
      "rgba(" +
      ((n & 16711680) >> 16) +
      "," +
      ((n & 65280) >> 8) +
      "," +
      (n & 255) +
      "," +
      t +
      ")"
    );
  };
  n.prototype.b2DebugDraw = function () {
    this.m_drawFlags = 0;
  };
  n.prototype.SetFlags = function (n) {
    void 0 === n && (n = 0);
    this.m_drawFlags = n;
  };
  n.prototype.GetFlags = function () {
    return this.m_drawFlags;
  };
  n.prototype.AppendFlags = function (n) {
    void 0 === n && (n = 0);
    this.m_drawFlags |= n;
  };
  n.prototype.ClearFlags = function (n) {
    void 0 === n && (n = 0);
    this.m_drawFlags &= ~n;
  };
  n.prototype.SetSprite = function (n) {
    this.m_ctx = n;
  };
  n.prototype.GetSprite = function () {
    return this.m_ctx;
  };
  n.prototype.SetDrawScale = function (n) {
    void 0 === n && (n = 0);
    this.m_drawScale = n;
  };
  n.prototype.GetDrawScale = function () {
    return this.m_drawScale;
  };
  n.prototype.SetLineThickness = function (n) {
    void 0 === n && (n = 0);
    this.m_lineThickness = n;
    this.m_ctx.strokeWidth = n;
  };
  n.prototype.GetLineThickness = function () {
    return this.m_lineThickness;
  };
  n.prototype.SetAlpha = function (n) {
    void 0 === n && (n = 0);
    this.m_alpha = n;
  };
  n.prototype.GetAlpha = function () {
    return this.m_alpha;
  };
  n.prototype.SetFillAlpha = function (n) {
    void 0 === n && (n = 0);
    this.m_fillAlpha = n;
  };
  n.prototype.GetFillAlpha = function () {
    return this.m_fillAlpha;
  };
  n.prototype.SetXFormScale = function (n) {
    void 0 === n && (n = 0);
    this.m_xformScale = n;
  };
  n.prototype.GetXFormScale = function () {
    return this.m_xformScale;
  };
  n.prototype.DrawPolygon = function (n, t, c) {
    if (t) {
      var h = this.m_ctx,
        l = this.m_drawScale;
      h.beginPath();
      h.strokeStyle = this._color(c.color, this.m_alpha);
      h.moveTo(n[0].x * l, n[0].y * l);
      for (c = 1; c < t; c++) h.lineTo(n[c].x * l, n[c].y * l);
      h.lineTo(n[0].x * l, n[0].y * l);
      h.closePath();
      h.stroke();
    }
  };
  n.prototype.DrawSolidPolygon = function (n, t, c) {
    if (t) {
      var h = this.m_ctx,
        l = this.m_drawScale;
      h.beginPath();
      h.strokeStyle = this._color(c.color, this.m_alpha);
      h.fillStyle = this._color(c.color, this.m_fillAlpha);
      h.moveTo(n[0].x * l, n[0].y * l);
      for (c = 1; c < t; c++) h.lineTo(n[c].x * l, n[c].y * l);
      h.lineTo(n[0].x * l, n[0].y * l);
      h.closePath();
      h.fill();
      h.stroke();
    }
  };
  n.prototype.DrawCircle = function (n, t, c) {
    if (t) {
      var h = this.m_ctx,
        l = this.m_drawScale;
      h.beginPath();
      h.strokeStyle = this._color(c.color, this.m_alpha);
      h.arc(n.x * l, n.y * l, t * l, 0, 2 * Math.PI, !0);
      h.closePath();
      h.stroke();
    }
  };
  n.prototype.DrawSolidCircle = function (n, t, c, h) {
    if (t) {
      var l = this.m_ctx,
        x = this.m_drawScale,
        e = n.x * x,
        w = n.y * x;
      l.moveTo(0, 0);
      l.beginPath();
      l.strokeStyle = this._color(h.color, this.m_alpha);
      l.fillStyle = this._color(h.color, this.m_fillAlpha);
      l.arc(e, w, t * x, 0, 2 * Math.PI, !0);
      l.moveTo(e, w);
      l.lineTo((n.x + c.x * t) * x, (n.y + c.y * t) * x);
      l.closePath();
      l.fill();
      l.stroke();
    }
  };
  n.prototype.DrawSegment = function (n, t, c) {
    var h = this.m_ctx,
      l = this.m_drawScale;
    h.strokeStyle = this._color(c.color, this.m_alpha);
    h.beginPath();
    h.moveTo(n.x * l, n.y * l);
    h.lineTo(t.x * l, t.y * l);
    h.closePath();
    h.stroke();
  };
  n.prototype.DrawTransform = function (n) {
    var t = this.m_ctx,
      c = this.m_drawScale;
    t.beginPath();
    t.strokeStyle = this._color(16711680, this.m_alpha);
    t.moveTo(n.position.x * c, n.position.y * c);
    t.lineTo(
      (n.position.x + this.m_xformScale * n.R.col1.x) * c,
      (n.position.y + this.m_xformScale * n.R.col1.y) * c
    );
    t.strokeStyle = this._color(65280, this.m_alpha);
    t.moveTo(n.position.x * c, n.position.y * c);
    t.lineTo(
      (n.position.x + this.m_xformScale * n.R.col2.x) * c,
      (n.position.y + this.m_xformScale * n.R.col2.y) * c
    );
    t.closePath();
    t.stroke();
  };
})();
var i;
for (i = 0; i < Box2D.postDefs.length; ++i) Box2D.postDefs[i]();
delete Box2D.postDefs;
