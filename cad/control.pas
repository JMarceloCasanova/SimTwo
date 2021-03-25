const
 NumJoints = 6;
 NumScrews = 1;
type
  faces = (fTop, fBottom, fLeft, fRight, fBack, fFront);
  controlModes = (cmManual, cmPaintModes, cmUDPServer);
  paintModes = (pmNone, pmBoxRaster);

  TTrajectory = record
    points : array [0..500] of TPoint3D;
    count: integer;
    nextPoint: integer;
    nextpointPerone: double;
  end;
// Global Variables
var

  irobot, iScrew, iB5, iB6, iHead: integer;
  d1, d2, d3: double;
  
  R01, R12, R23: Matrix;
  R03: Matrix;

  JointPos: array[0..NumJoints - 1] of double;
  ScrewH: array[0..NumScrews - 1] of double;

  state: string;
  ReqThetas: matrix;
  Tol, tis: double;

  //JM
  lr_mode: integer;
  ud_mode: integer;

  controlMode: controlModes;
  paintMode: paintModes;

  ext: TExtents;
  traj1 : TTrajectory;
  //spray gun
  //sg_x, sg_y, sg_z,
  sg : TPoint3D;
  sg_theta: double;

  connection: Boolean;
  await_synack: Boolean;

function diffTPoint3D(a, b:TPoint3D):TPoint3D;
begin
  result.x := a.x - b.x;
  result.y := a.y - b.y;
  result.z := a.z - b.z;
end;
function addTPoint3D(a, b:TPoint3D):TPoint3D;
begin
  result.x := a.x + b.x;
  result.y := a.y + b.y;
  result.z := a.z + b.z;
end;
function scaleTPoint3D(a:TPoint3D; scale: double):TPoint3D;
begin
  result.x := a.x * scale;
  result.y := a.y * scale;
  result.z := a.z * scale;
end;
function sizeTPoint3D(a:TPoint3D):double;
begin
  result := sqrt(a.x*a.x+a.y*a.y+a.z*a.z);
end;
function distTPoint3D(a, b:TPoint3D):double;
begin
  result := sizeTPoint3D(diffTPoint3D(a,b));
end;
function normTPoint3D(a:TPoint3D):TPoint3D;
var dist: double;
begin
  dist := sizeTPoint3D(a);
  result.x := a.x/dist;
  result.y := a.y/dist;
  result.z := a.z/dist;
end;

function DHMat(a, alpha, d, theta: double): Matrix;
var ct, st, ca, sa: double;
    R: Matrix;
begin
  ct := cos(theta);
  st := sin(theta);
  ca := cos(alpha);
  sa := sin(alpha);
  
  R := Meye(4);
  MSetV(R, 0, 0, ct); MSetV(R, 0, 1,-st * ca);  MSetV(R, 0, 2, st * sa);  MSetV(R, 0, 3, a * ct);
  MSetV(R, 1, 0, st); MSetV(R, 1, 1, ct * ca);  MSetV(R, 1, 2,-ct * sa);  MSetV(R, 1, 3, a * st);
  MSetV(R, 2, 0,  0); MSetV(R, 2, 1, sa     );  MSetV(R, 2, 2, ca     );  MSetV(R, 2, 3, d     );
  result := R;
end;


function RotZMat(theta: double): Matrix;
var ct, st: double;
    R: Matrix;
begin
  ct := cos(theta);
  st := sin(theta);

  R := Meye(3);
  MSetV(R, 0, 0, ct); MSetV(R, 0, 1,-st);  MSetV(R, 0, 2, 0);
  MSetV(R, 1, 0, st); MSetV(R, 1, 1, ct);  MSetV(R, 1, 2, 0);
  MSetV(R, 2, 0,  0); MSetV(R, 2, 1, 0 );  MSetV(R, 2, 2, 1);

  result := R;
end;


function RotXMat(theta: double): Matrix;
var ct, st: double;
    R: Matrix;
begin
  ct := cos(theta);
  st := sin(theta);

  R := Meye(3);
  MSetV(R, 0, 0, 1 ); MSetV(R, 0, 1, 0 );  MSetV(R, 0, 2, 0  );
  MSetV(R, 1, 0, 0 ); MSetV(R, 1, 1, ct);  MSetV(R, 1, 2, -st);
  MSetV(R, 2, 0,  0); MSetV(R, 2, 1, st);  MSetV(R, 2, 2, ct );

  result := R;
end;

procedure UpdateScrew(index: integer);
var w: double;
begin
  w := GetAxisSpeed(index, 1); // second axis has the head rotation
  SetRCValue(3, 4, format('%.2g',[w]));
  if abs(w) > 0.1 then begin
    ScrewH[index - 1] := ScrewH[index - 1] + w/1000;
    SetAxisPosRef(index, 0, ScrewH[index - 1]);
  end;

end;


// Pace here the Inverse Kinematics calculations
function IK(Xtool, Rtool: matrix; Ltool: double): matrix;
begin
  result := Mzeros(6, 1);
end;



procedure SetThetas(Thetas: matrix);
var i: integer;
begin
  for i := 0 to 5 do begin
    SetAxisPosRef(iRobot, i, Mgetv(Thetas, i, 0));
  end;
end;


function JointError(ReqThethas: matrix): double;
var err: double;
    i: integer;
begin
  err := 0;
  for i := 0 to 5 do begin
    err := err + abs(GetAxisPos(iRobot, i) - Mgetv(ReqThethas, i, 0));
  end;
  result := err;
end;


procedure SetNewState(newState: string);
begin
  State := newState;
  tis := 0;
end;

procedure ManualControl();
begin

  if RCButtonPressed(6, 9) then lr_mode:=1;
  if RCButtonPressed(6, 10) then lr_mode:=3;
  if RCButtonPressed(6, 11) then lr_mode:=5;
  if RCButtonPressed(6, 12) then lr_mode:=7;
  if RCButtonPressed(7, 9) then ud_mode:=2;
  if RCButtonPressed(7, 10) then ud_mode:=4;
  if RCButtonPressed(7, 11) then ud_mode:=6;
  if RCButtonPressed(7, 12) then ud_mode:=8;

  case lr_mode of
    1: begin
        if KeyPressed(vk_left) then begin
          sg.x := sg.x-0.05;
        end else if KeyPressed(vk_right) then begin
          sg.x := sg.x+0.05;
        end
      end;
    3: begin
        if KeyPressed(vk_left) then begin
          sg.y := sg.y-0.05;
        end else if KeyPressed(vk_right) then begin
          sg.y := sg.y+0.05;
        end
      end;
    5: begin
        if KeyPressed(vk_left) then begin
          sg.z := sg.z-0.05;
        end else if KeyPressed(vk_right) then begin
          sg.z := sg.z+0.05;
        end
      end;
    7: begin
        if KeyPressed(vk_left) then begin
          sg_theta := sg_theta+0.05;
        end else if KeyPressed(vk_right) then begin
          sg_theta := sg_theta-0.05;
        end
      end;
  end;
  case ud_mode of
    2: begin
        if KeyPressed(vk_down) then begin
          sg.x := sg.x-0.05;
        end else if KeyPressed(vk_up) then begin
          sg.x := sg.x+0.05;
        end
      end;
    4: begin
        if KeyPressed(vk_down) then begin
          sg.y := sg.y-0.05;
        end else if KeyPressed(vk_up) then begin
          sg.y := sg.y+0.05;
        end
      end;
    6: begin
        if KeyPressed(vk_down) then begin
          sg.z := sg.z-0.05;
        end else if KeyPressed(vk_up) then begin
          sg.z := sg.z+0.05;
        end
      end;
    8: begin
        if KeyPressed(vk_down) then begin
          sg_theta := sg_theta-0.05;
        end else if KeyPressed(vk_up) then begin
          sg_theta := sg_theta+0.05;
        end
      end;
    end;

end;

procedure EncodeInteger(var StrPacket: TStringList; name: string; data: integer);
begin
  StrPacket.add(name);
  StrPacket.add(format('%d',[data]));
  StrPacket.add('');
end;

procedure EncodeDouble(var StrPacket: TStringList; name: string; data: double);
begin
  StrPacket.add(name);
  StrPacket.add(format('%.6g',[data]));
  StrPacket.add('');
end;

function DecodeDoubleDef(var StrPacket: TStringList; name: string; defval: double): double;
var i: integer;
begin
  result := defval;
  i := StrPacket.indexof(name);
  if (i < 0) or (i + 1 >= StrPacket.count) then exit;
  result := strtofloat(StrPacket[i+1]);
end;

procedure sendUdp(msg: string);
begin
  WriteUDPData(GetRCText(1,16), strtoint(GetRCText(2,16)), msg);
end;

procedure ServerControl();
var StrPacket: TStringList;
    txt: string;
    i: integer;
    a: double;
begin

  if connection then begin
    StrPacket := TStringList.create;
    try
      //EncodeInteger(StrPacket,'Enc1', GetAxisOdo(0, 0));

      for i := 0 to 3 do begin
        //EncodeDouble(StrPacket, 's' + inttostr(i), GetSensorVal(0, i));
      end;

      //WriteUDPData(GetRCText(1,16), strtoint(GetRCText(2,16)), StrPacket.text);

      while true do begin
        StrPacket.text := ReadUDPData();
        txt := StrPacket.text;
        WriteLn(txt);
        if txt = '' then break;
        // Read Motor Speed Reference
        a := DecodeDoubleDef(StrPacket, 'a', 0);
      end;
    finally
      StrPacket.free;
    end;  
  end else begin
    while true do begin
        txt := ReadUDPData();
        if await_synack and (txt = 'SYNACK') then begin
          connection := true;
        end else if txt = 'SYN' then begin
          sendUdp('ACK');
          await_synack := True;
        end;
        if txt = '' then break;
      end;


  end;

end;

procedure RunTrajectory1(vel: double);
var dist: double;
    dirnorm: TPoint3D;
begin
  if traj1.nextPoint = 0 then begin
    traj1.nextPoint := 1;
    traj1.nextpointPerone := 0;
    sg := traj1.points[0];
  end else if traj1.nextPoint > traj1.count then begin
    paintMode := pmNone;
  end;

  if traj1.nextpointPerone>0.95 then begin
    traj1.nextPoint := traj1.nextPoint+1;
    traj1.nextpointPerone := 0;
  end;

  dirnorm := normTPoint3D(diffTPoint3D(traj1.points[traj1.nextPoint], traj1.points[traj1.nextPoint-1]));
  sg := AddTPoint3D(sg, scaleTPoint3D(dirnorm,vel));
  traj1.nextpointPerone := distTPoint3D(sg, traj1.points[traj1.nextPoint-1])/distTPoint3D(traj1.points[traj1.nextPoint], traj1.points[traj1.nextPoint-1]);

end;

procedure DrawTrajectory(traj: TTrajectory);
var i, n: integer;
begin
  ClearTrail(0);
  SetTrailColor(0, 0, 255, 0);
  n := traj.count;
  if n>95 then begin
    n := 95;
    WriteLn('trajectory points, exceeds cell number');
  end;
  for i:= 0 to (n-1) do begin
    AddTrailNode(0, traj.points[i].X, traj.points[i].Y, traj.points[i].z);
    SetRCValue(2,21+i, '[go]');
    SetRCValue(3,21+i, FloatToStr(traj.points[i].X));
    SetRCValue(4,21+i, FloatToStr(traj.points[i].Y));
    SetRCValue(5,21+i, FloatToStr(traj.points[i].Z));
  end;
  SetRCValue(1, 22, IntToStr(traj.count));
  SetRCValue(3,21, FloatToStr(traj.points[0].X));
  SetRCValue(4,21, FloatToStr(traj.points[0].Y));
  SetRCValue(5,21, FloatToStr(traj.points[0].Z));
  sg := traj.points[0];
end;

function CalculateBoxRaster(BoxSelectFace: faces; BoxOffset: double; BoxUStep, BoxVExtend: double): TTrajectory;
var traj: TTrajectory;
    faceExt: TExtents;
    i: integer;
begin
  if BoxSelectFace = fTop then begin
    faceExt := ext;
    faceExt.min.Z := ext.max.Z + BoxOffset;
    faceExt.max.Z := ext.max.Z + BoxOffset;
    faceExt.min.X := faceExt.min.X - BoxUStep;
    faceExt.max.X := faceExt.max.X + BoxUStep;
    faceExt.min.Y := faceExt.min.Y - BoxVExtend;
    faceExt.max.Y := faceExt.max.Y + BoxVExtend;
  end;
  traj.count := 2*round((faceExt.max.X - faceExt.min.X) / BoxUStep) + 2;
  for i:= 0 to traj.count-1 do begin
    traj.points[i*4].X := faceExt.min.X + i*2*BoxUStep;
    traj.points[i*4].Y := faceExt.min.Y;
    traj.points[i*4].Z := faceExt.max.Z;
    traj.points[i*4+1].X := faceExt.min.X + i*2*BoxUStep;
    traj.points[i*4+1].Y := faceExt.max.Y;
    traj.points[i*4+1].Z := faceExt.max.Z;
    traj.points[i*4+2].X := faceExt.min.X + (i*2+1)*BoxUStep;
    traj.points[i*4+2].Y := faceExt.max.Y;
    traj.points[i*4+2].Z := faceExt.max.Z;
    traj.points[i*4+3].X := faceExt.min.X + (i*2+1)*BoxUStep;
    traj.points[i*4+3].Y := faceExt.min.Y;
    traj.points[i*4+3].Z := faceExt.max.Z;
  end;
  result := traj;
end;


procedure Control;
var i: integer;
    B5Pos, B6Pos: Matrix;
    B6Rot, B6RotCalc: Matrix;
    xw, yw, zw: double;
    RTool, XTool, XWrist, LTool: matrix;
    HeadPos, HeadRot: matrix;

    BoxSelectFace: faces;
    BoxOffset: double;
    BoxUStep, BoxVExtend: double;
    traj: TTrajectory;
begin
//jm
  if RCButtonPressed(6, 4) then ResetPaintTargetPaint(0);
  if RCButtonPressed(7, 4) then SetPaintTargetPaintMode(0, pmPaint);
  if RCButtonPressed(8, 4) then SetPaintTargetPaintMode(0, pmHeatmap);
  if RCButtonPressed(10, 4) then SetSprayGunOn(0);
  if RCButtonPressed(11, 4) then SetSprayGunOff(0);


  if RCButtonPressed(6, 13) then begin
    sg := ext.Max;
  end;
  if RCButtonPressed(7, 13) then begin
    sg := ext.Min;
  end;

  if RCButtonPressed(1,9) then controlMode := cmManual;
  if RCButtonPressed(1,10) then controlMode := cmPaintModes;
  if RCButtonPressed(1,11) then controlMode := cmUDPServer;

  if controlMode = cmPaintModes then begin
    SetRCValue(2,8,'PaintModes');
    SetRCBackColor(2, 10, $0000FF00);
    SetRCBackColor(2, 9, $7FFFFFFF);
    SetRCBackColor(2,11, $7FFFFFFF);
    SetRCBackColor(3, 8, $7FFFFFFF);
    if RCButtonPressed(3,9) then begin
      paintMode := pmBoxRaster;
      BoxSelectFace := fTop;
      BoxOffset := 0.4;
      BoxUStep := 0.1;
      BoxVExtend := 0.3;
      traj := CalculateBoxRaster(BoxSelectFace, BoxOffset, BoxUStep, BoxVExtend);
      traj1 := traj;
      DrawTrajectory(traj);
    end;
    if paintMode = pmBoxRaster then begin
      SetRCValue(4, 8, 'Box Raster');
      for i:=0 to traj1.count-1 do begin
        if RCButtonPressed(2,21+i) then begin
          sg := traj1.points[i];
        end;
      end;
      //WriteLn('e');
      RunTrajectory1(0.015);
    end;
  end else if controlMode = cmManual then begin
    SetRCValue(2,8,'Manual');
    SetRCBackColor(2, 9, $7F00FFFF);
    SetRCBackColor(2,10, $7FFFFFFF);
    SetRCBackColor(2,11, $7FFFFFFF);
    SetRCBackColor(3, 8, $7FAAAAAA);
    ManualControl();
  end else if controlMode = cmUDPServer then begin
    SetRCValue(2,8,'UDPServer');
    if connection then begin 
      SetRCValue(2,7,'Connected');
    end else begin
      SetRCValue(2,7,'Not Connected');
    end; 
    SetRCBackColor(2, 9, $7FFFFFFF);
    SetRCBackColor(2,10, $7FFFFFFF);
    SetRCBackColor(2,11, $7F00FFFF);
    SetRCBackColor(3, 8, $7FAAAAAA);
    ServerControl();
  end;

  SetRobotPos(0, sg.x, sg.y, sg.z, sg_theta);
  //SetRobotPos(2, sg.x, sg.y, sg.z, sg_theta);
  //end jm

  {UpdateScrew(1);

  B5Pos := GetSolidPosMat(iRobot, iB5);
  MatrixToRange(11, 2, B5Pos);

  B6Pos := GetSolidPosMat(iRobot, iB6);
  MatrixToRange(16, 2, B6Pos);

  B6rot := GetSolidRotMat(iRobot, iB6);
  MatrixToRangeF(16, 4, B6Rot, '%.3f');

  HeadPos := GetSolidPosMat(iScrew, iHead);
  HeadRot := GetSolidRotMat(iScrew, iHead);


  // Read joint positions
  for i := 0 to NumJoints -1 do begin
    JointPos[i] := GetAxisPos(irobot, i);
  end;

  // and show
  for i := 0 to NumJoints -1 do begin
    SetRCValue(3 + i, 2, format('%.3g',[Deg(JointPos[i])]));
  end;

  if RCButtonPressed(2, 4) then begin
    SetRobotPos(iScrew, 0.4 + (random01()- 0.1) * 0.2, -0.1 + (random01()- 0.1) * 0.2, 0, 0);
  end;

  // control equations
  // ...

  if (state = 'idle') and RCButtonPressed(10,4) then begin
    SetNewState('above');
  end else if (state = 'above') and (JointError(ReqThetas) < Tol) and (tis > 5.0) then begin
    SetNewState('lock');
  end;

  if state = 'idle' then begin
    ReqThetas := Mzeros(6, 1);
    SetThetas(ReqThetas);
  end else if state = 'above' then begin
    ReqThetas := Ik(HeadPos, HeadRot, 0.15);
    SetThetas(ReqThetas);
  end else if state = 'lock' then begin
    ReqThetas := Ik(HeadPos, HeadRot, 0.8);
    SetThetas(ReqThetas);
  end;

}
  tis := tis + 0.04;

  SetRCValue(3, 7, state);
end;


procedure Initialize;
var i: integer;
begin
  irobot := 0;
  iScrew := 1;

  {iB5 := GetSolidIndex(irobot, 'B5');
  iB6 := GetSolidIndex(irobot, 'B6');

  iHead := GetSolidIndex(iScrew, 'screw_head');

  SetRCValue(2, 1, 'Joint');
  SetRCValue(2, 2, 'Pos (deg)');
  for i := 0 to NumJoints -1 do begin
    SetRCValue(3 + i, 1, format('%d',[i]));
  end;

  for i := 0 to NumScrews -1 do begin
    Screwh[i] := GetAxisPos(1 + i, 0);
  end;

  d1 := 0.55;
  d2 := 0.4;
  d3 := 0.37;
}
  state := 'idle';
  Tol := 0.2;

  //JM
  lr_mode:=1;
  ud_mode:= 4;
  //spray gun
  sg.x:=0;
  sg.y:=-1.5;
  sg.z:=0.6;
  sg_theta:=0;
  controlMode := cmManual;
  paintMode := pmNone;

  ext := GetPaintTargetExtents(0);

  SetRCValue(11, 8, format('%.2f',[ext.Max.x]));
  SetRCValue(11, 9, format('%.2f',[ext.Max.y]));
  SetRCValue(11, 10, format('%.2f',[ext.Max.z]));
  SetRCValue(12, 8, format('%.2f',[ext.Min.x]));
  SetRCValue(12, 9, format('%.2f',[ext.Min.y]));
  SetRCValue(12, 10, format('%.2f',[ext.Min.z]));

  connection := False;
  await_synack := False;
  SetRCValue(2,7,'Not Connected');
end;
