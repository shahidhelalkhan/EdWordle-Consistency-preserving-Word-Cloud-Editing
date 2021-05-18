var firstshow = !0,
  Allfont =
    "HoboStd;Kristen ITC;Bernard MT Condensed;Expressway Free;MS Reference Sans Serif;Cooper Std Black;Gunplay;Coolvetica;LucidaSans-Bold;Mailraystuff;Teen".split(
      ";"
    );
$(function (a) {
  var b = new dhtmlXColorPicker({
    input: "wordColor",
    custom_colors: color[colorthemenum],
  });
  b.setPosition("right");
  b.attachEvent("onChange", function (a) {
    var b = selectedBody.GetID();
    tagColor[b] = a;
  });
  b.attachEvent("onSelect", function (a) {
    var b = selectedBody.GetID();
    tagColor[b] = a;
  });
  b.attachEvent("onCancel", function () {
    var a = document.getElementById("wordColor").style.backgroundColor,
      b = selectedBody.GetID();
    tagColor[b] = a;
    document.getElementById("wordColor").value = "word color";
    return !0;
  });
  var c = new dhtmlXColorPicker({
    input: "wordsColor",
    custom_colors: color[colorthemenum],
  });
  c.setPosition("top");
  c.attachEvent("onChange", function (a) {
    for (var b = 0; b < multibodies.length; b++) tagColor[multibodies[b]] = a;
  });
  c.attachEvent("onSelect", function (a) {
    for (var b = 0; b < multibodies.length; b++) tagColor[multibodies[b]] = a;
  });
  a("#closeBtn")
    .hover(
      function () {
        a(this).css({ color: "black" });
      },
      function () {
        a(this).css({ color: "#999" });
      }
    )
    .on("click", function () {
      a("#LoadBox").fadeOut("fast");
    });
  a("#closeBtn2")
    .hover(
      function () {
        a(this).css({ color: "black" });
      },
      function () {
        a(this).css({ color: "#999" });
      }
    )
    .on("click", function () {
      a("#ShowWordsList").fadeOut("fast");
    });
  a("#closeBtn3")
    .hover(
      function () {
        a(this).css({ color: "black" });
      },
      function () {
        a(this).css({ color: "#999" });
      }
    )
    .on("click", function () {
      a("#AddWordWindow").fadeOut("fast");
    });
  a(".dropdown dt a").click(function () {
    a(".dropdown dd ul").toggle();
  });
  a(".dropdown dd ul li a").click(function () {
    var d = a(this).html();
    a(".dropdown dt a span").html(d);
    a(".dropdown dd ul").hide();
    colorthemenum = parseInt(a("#sample").find("dt a span.value").html());
    2 == colorthemenum
      ? (document.getElementById("backgroudColor").jscolor.fromString("454545"),
        (backgroundcolor = "#454545"))
      : 6 == colorthemenum
      ? (document.getElementById("backgroudColor").jscolor.fromString("212121"),
        (backgroundcolor = "#212121"))
      : (document.getElementById("backgroudColor").jscolor.fromString("FFFFFF"),
        (backgroundcolor = "#ffffff"));
    b = new dhtmlXColorPicker({
      input: "wordColor",
      custom_colors: color[colorthemenum],
    });
    b.setPosition("right");
    b.attachEvent("onChange", function (a) {
      var b = selectedBody.GetID();
      tagColor[b] = a;
    });
    b.attachEvent("onSelect", function (a) {
      var b = selectedBody.GetID();
      tagColor[b] = a;
    });
    b.attachEvent("onCancel", function () {
      var a = document.getElementById("wordColor").style.backgroundColor,
        b = selectedBody.GetID();
      tagColor[b] = a;
      document.getElementById("wordColor").value = "word color";
      return !0;
    });
    c = new dhtmlXColorPicker({
      input: "wordColor",
      custom_colors: color[colorthemenum],
    });
    c.setPosition("right");
    c.attachEvent("onChange", function (a) {
      var b = selectedBody.GetID();
      tagColor[b] = a;
    });
    c.attachEvent("onSelect", function (a) {
      var b = selectedBody.GetID();
      tagColor[b] = a;
    });
    c.attachEvent("onCancel", function () {
      var a = document.getElementById("wordColor").style.backgroundColor,
        b = selectedBody.GetID();
      tagColor[b] = a;
      document.getElementById("wordColor").value = "word color";
      return !0;
    });
    for (d = 0; d < tagColor.length; d++) tagColor[d] = createColor();
  });
  a(document).bind("click", function (b) {
    a(b.target).parents().hasClass("dropdown") || a(".dropdown dd ul").hide();
  });
  var d = !1,
    e,
    f;
  var h = (a(document.body).width() - 280) / 2;
  var g = parseInt(a("#ShowWordsList").css("top"));
  document.getElementById("words_list_title").onmousedown = function (a) {
    d = !0;
    e = a.clientX;
    f = a.clientY;
  };
  document.onmousemove = function (b) {
    if (d) {
      var c = b.clientY - f + g + "px";
      b = b.clientX - e + h + "px";
      a("#ShowWordsList").css("top", c);
      a("#ShowWordsList").css("left", b);
    }
  };
  document.onmouseup = function () {
    d &&
      ((h = parseInt(a("#ShowWordsList").css("left"))),
      (g = parseInt(a("#ShowWordsList").css("top"))),
      (d = !1));
  };
});
function LoadFile() {
  document.getElementById("txt").value = "";
  var a = !0;
  void 0 != wo &&
    (a =
      1 ==
      confirm("Are you sure to Load a new data? Maybe you should save first.")
        ? !0
        : !1);
  a &&
    (DestroyBody(!0),
    (preEditInfo = []),
    (wordnum = preEditPoint = 0),
    (hasselect.flag = !1),
    $("#LoadBox").fadeIn("slow"));
}
function SaveImg() {
  var a = document.getElementById("canvas").toDataURL("image/png"),
    b = document.createElementNS("http://www.w3.org/1999/xhtml", "a");
  b.href = a;
  b.download = "mypic.png";
  a = document.createEvent("MouseEvents");
  a.initMouseEvent(
    "click",
    !0,
    !1,
    window,
    0,
    0,
    0,
    0,
    0,
    !1,
    !1,
    !1,
    !1,
    0,
    null
  );
  b.dispatchEvent(a);
}
function SavePdf() {
  console.log("save pdf");
  var a = canvas.toDataURL("image/jpeg", 1),
    b = new jsPDF({ orientation: "landscape" });
  b.addImage(a, "JPEG", 0, 0);
  document.getElementById("download");
  b.save("download.pdf");
}
function ShowWords() {
  $("#ShowWordsList").fadeIn("slow");
  if (firstshow) {
    var a = (document.body.clientWidth - 280) / 2 + "px",
      b = (document.body.clientHeight - 410) / 2 + "px";
    $("#ShowWordsList").css("top", b);
    $("#ShowWordsList").css("left", a);
    firstshow = !1;
  }
  CreateWordTable();
}
function CreateWordTable() {
  $("#words_list").empty("");
  var a = document.getElementById("words_list");
  var b = document.createElement("tbody");
  a.appendChild(b);
  var c = b.insertRow(0);
  a = ["word", "weight"];
  for (var d = 0; d < a.length; d++) {
    var e = document.createElement("th");
    e.innerHTML = a[d];
    c.appendChild(e);
  }
  for (d = 0; d < wordnum; d++)
    for (
      c = b.insertRow(b.rows.length),
        c.setAttribute("id", d),
        c.setAttribute("onclick", "selectLineForDelete(this)"),
        -1 != multibodies.indexOf(d) && (c.bgColor = "#FFFF77"),
        a = [wo[d], we[d].toFixed(2)],
        e = 0;
      2 > e;
      e++
    )
      c.insertCell(c.cells.length).innerHTML = a[e];
}
function selectLineForDelete(a) {
  if ("undefined " == typeof a) alert("\u7f3a\u5c11\u53c2\u6570");
  else {
    var b = parseInt(a.id);
    -1 != multibodies.indexOf(b) ? multibodies.remove(b) : multibodies.push(b);
    a.bgColor = "" == a.bgColor ? "#FFFF77" : "";
  }
}
function DeleteWords() {
  SavePreEditPosition();
  multibodies.sort(sortNumber);
  for (var a = 0; a < multibodies.length; a++)
    DeleteWord(!1, multibodies[a] - a);
  multibodies = [];
  $("#ShowWordsList").fadeOut("fast");
}
function SetFontforSelect() {
  for (
    var a = document.getElementById("wordsFont"),
      a = Allfont[a.options[a.selectedIndex].value],
      b = 0;
    b < multibodies.length;
    b++
  )
    SetWordFont(!0, multibodies[b], a);
}
function ShowAddWord() {
  $("#AddWordWindow").fadeIn("slow");
  removable = !0;
}
function AddWord() {
  var a = !0;
  80 > iternum && (a = !1);
  if (a) {
    SavePreEditPosition();
    var b = document.getElementById("add_word").value,
      c = parseFloat(document.getElementById("add_weight").value);
    if (
      "" != b &&
      -1 == b.indexOf(" ") &&
      null != b &&
      void 0 != b &&
      0 <= c &&
      1 >= c
    ) {
      for (var d = (a = 0); d < wordnum; d++)
        if (we[d] < c) {
          a = d;
          break;
        }
      wo.splice(a, 0, b);
      we.splice(a, 0, c);
      bodyList.splice(a, 0, void 0);
      tagColor.splice(a, 0, void 0);
      tagFont.splice(a, 0, void 0);
      bodywidth.splice(a, 0, void 0);
      bodyheight.splice(a, 0, void 0);
      bodydesent.splice(a, 0, void 0);
      ifattract.splice(a, 0, void 0);
      for (d = 0; d < bodyList.length; d++) bodyList[d] && bodyList[d].SetID(d);
      b = createOffSet();
      c = createOffSet();
      createBox(canvaswidth / 2 + 1 * b, canvasheight / 2 + 1 * c, we[a], a);
      wordnum++;
      ifstart = !1;
      $("#AddWordWindow").fadeOut("fast");
    } else alert("The input is incorrect!");
  } else alert("Please wait for the system to stop automatically.");
}
function AddUserForce() {
  if (UserAddForce) {
    UserAddForce = !1;
    document.getElementById("force_switch").innerHTML = "Force On";
    for (var a = 0; a < bodyList.length; a++)
      bodyList[a].SetAngularVelocity(0),
        bodyList[a].SetLinearVelocity(new b2Vec2(0, 0));
  } else
    (UserAddForce = !0),
      (document.getElementById("force_switch").innerHTML = "Force Off"),
      (iternum = 0);
}
function DoReWordle() {
  UserReWordle = !0;
}
function SetBackColor() {
  backgroundcolor = "#" + document.getElementById("backgroudColor").value;
}
function SetFontforAll() {
  console.log("font");
  var a = document.getElementById("setallfont"),
    a = Allfont[a.options[a.selectedIndex].value];
  if (bodyList[0]) {
    hasselect.flag && (hasselect.flag = !1);
    defaultfont = a;
    for (var b = SaveNow(), c = 0; c < tagFont.length; c++) tagFont[c] = a;
    DestroyBody(!1);
    ifattract = b.wordsIfatr;
    tagColor = b.wordsTagColor;
    ReCreateObject_Load(
      b.wordsPosition,
      b.wordsAngle,
      b.wordsIffix,
      b.wordsiternum
    );
    iternum = 0;
  } else defaultfont = a;
}
function DeleteWord(a, b) {
  var c = a ? selectedBody.GetID() : b;
  drawnBody.remove(bodyList[c]);
  hasselect.flag = !1;
  var d = [].concat(wo);
  wo.splice(wordnum, 0, wo[c]);
  wo.splice(c, 1);
  we.splice(wordnum, 0, we[c]);
  we.splice(c, 1);
  bodyList.splice(wordnum, 0, bodyList[c]);
  bodyList.splice(c, 1);
  for (var e = 0; e < bodyList.length; e++) bodyList[e].SetID(e);
  for (e = bodyList[wordnum - 1].GetFixtureList(); e; e = e.m_next)
    e.SetSensor(!0);
  var e = {
      x: bodyList[wordnum - 1].GetPosition().x,
      y: bodyList[wordnum - 1].GetPosition().y,
    },
    f = { x: 0, y: 0 };
  f.x = (750 + 2 * createOffSet()) / drawScale;
  300 >= 30 * bodyList[wordnum - 1].GetPosition().y
    ? (f.y = -150 / drawScale)
    : (f.y = 750 / drawScale);
  animation.push({
    id: wordnum - 1,
    startpos: e,
    endpos: f,
    count: 0,
    editflag: 1,
  });
  tagColor.splice(wordnum, 0, tagColor[c]);
  tagColor.splice(c, 1);
  tagFont.splice(wordnum, 0, tagFont[c]);
  tagFont.splice(c, 1);
  bodywidth.splice(wordnum, 0, bodywidth[c]);
  bodywidth.splice(c, 1);
  bodyheight.splice(wordnum, 0, bodyheight[c]);
  bodyheight.splice(c, 1);
  bodydesent.splice(wordnum, 0, bodydesent[c]);
  bodydesent.splice(c, 1);
  prePosition.splice(c, 1);
  ifattract.push(!1);
  ifattract.splice(c, 1);
  wordsNeighbor.splice[(c, 1)];
  wordnum--;
  for (e = 0; e < constrainBody.length; e++)
    for (c = 0; c < constrainBody[e].length; c++)
      (f = wo.indexOf(d[constrainBody[e][c]])),
        -1 != f && (constrainBody[e][c] = f);
  iternum = 0;
  a &&
    (handleMouseUp(),
    (hasselect.flag = !1),
    (hasselect.time = 0),
    (preBodypos = premouseY = premouseX = mouseY = mouseX = void 0),
    (firstrotate = !0),
    (mrRadian = selectedBody = preselectBody = void 0),
    (document.getElementById("wordedittools").style.display = "none"));
}
function SetWordFont(a, b, c) {
  a ||
    ((b = selectedBody.GetID()),
    (c = document.getElementById("wordfont")),
    (c = Allfont[c.options[c.selectedIndex].value]));
  var d = {};
  d.x = bodyList[b].GetPosition().x;
  d.y = bodyList[b].GetPosition().y;
  var e = bodyList[b].GetAngle(),
    f = bodyList[b].IsFixedRotation();
  tagFont[b] = c;
  world.DestroyBody(bodyList[b]);
  ReCreateBody(d, e, f, b);
  a || ((selectedBody = bodyList[b]), (preselectBody = bodyList[b]));
}
function SeeBox() {
  seebox = seebox ? !1 : !0;
}
function KeyUp(a) {
  if (hasselect.flag) {
    var b = selectedBody.GetID(),
      c = 1.05 * we[b];
    if (0.5 > c || (0.5 <= c && 1 == ifsubdivided[b])) ScaleWords(b, c);
    else {
      var d = {
          x: bodyList[b].GetPosition().x,
          y: bodyList[b].GetPosition().y,
        },
        e = bodyList[b].GetAngle(),
        f = bodyList[b].IsFixedRotation();
      world.DestroyBody(bodyList[b]);
      we[b] = c;
      ReCreateBody(d, e, f, b, !1, !0);
      ifsubdivided[b] = !0;
    }
    ifattract[b] && (iternum = 0);
  }
  a.preventDefault && a.preventDefault();
}
function KeyDown(a) {
  if (hasselect.flag) {
    var b = selectedBody.GetID();
    ScaleWords(b, 0.95 * we[b]);
    ifattract[b] && (iternum = 0);
  }
  a.preventDefault && a.preventDefault();
}
function KeyRotate(a) {
  a = a ? 0.1 : -0.1;
  if (0 < multibodies.length) {
    for (var b = multicenter, c = [], d = 0; d < multibodies.length; d++) {
      var e = multibodies[d];
      premulpos[d] = {
        x: bodyList[e].GetPosition().x,
        y: bodyList[e].GetPosition().y,
      };
      c[d] = bodyList[e].GetAngle();
    }
    if (0 != multibodies.length)
      for (d = 0; d < multibodies.length; d++) {
        e = multibodies[d];
        var f = premulpos[d].x,
          h = premulpos[d].y,
          g = a,
          k = (f - b.x) * Math.cos(g) - (h - b.y) * Math.sin(g) + b.x,
          h = (f - b.x) * Math.sin(g) + (h - b.y) * Math.cos(g) + b.y,
          f = k;
        bodyList[e].SetPosition(new b2Vec2(f, h));
        bodyList[e].SetAngle(g + c[d]);
      }
  } else
    selectedBody.SetAngularVelocity(0),
      selectedBody.SetLinearVelocity(new b2Vec2(0, 0)),
      selectedBody.SetAwake(!0),
      selectedBody.m_force.SetZero(),
      (e = selectedBody.GetAngle()),
      selectedBody.SetAngle(e + a);
}
Array.prototype.remove = function (a) {
  a = this.indexOf(a);
  -1 < a && this.splice(a, 1);
};
