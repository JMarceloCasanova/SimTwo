const
 NumJoints = 6;
 NumScrews = 1;
type
  faces = (fTop, fBottom, fLeft, fRight, fBack, fFront);
  controlModes = (cmManual, cmPaintModes);
  paintModes = (pmNone, pmBoxRaster);

  TTrajectory = record
    points : array [0..500] of TPoint3D;

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

  //spray gun
  sg_x, sg_y, sg_z, sg_theta: double;

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

procedure BoxRasterPaint(BoxSelectFace: faces; BoxOffset: double;BoxAngle: double;
    BoxUStep: double);
begin

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
    BoxAngle: double;
    BoxUStep, BoxVStep: double;
begin
//jm
  if RCButtonPressed(6, 9) then lr_mode:=1;
  if RCButtonPressed(6, 10) then lr_mode:=3;
  if RCButtonPressed(6, 11) then lr_mode:=5;
  if RCButtonPressed(6, 12) then lr_mode:=7;
  if RCButtonPressed(7, 9) then ud_mode:=2;
  if RCButtonPressed(7, 10) then ud_mode:=4;
  if RCButtonPressed(7, 11) then ud_mode:=6;
  if RCButtonPressed(7, 12) then ud_mode:=8;

  if RCButtonPressed(6, 13) then begin
    sg_x := ext.Max.x;
    sg_y := ext.Max.y;
    sg_z := ext.Max.z;
  end;
  if RCButtonPressed(7, 13) then begin
    sg_x := ext.Min.x;
    sg_y := ext.Min.y;
    sg_z := ext.Min.z;
  end;

  if RCButtonPressed(1,9) then controlMode := cmManual;
  if RCButtonPressed(1,10) then controlMode := cmPaintModes;

  if controlMode = cmPaintModes then begin
    SetRCValue(2,8,'PaintModes');
    SetRCBackColor(2, 10, $0000FF00);
    SetRCBackColor(2, 9, $7FFFFFFF);
    SetRCBackColor(3, 8, $7FFFFFFF);
    if RCButtonPressed(3,9) then begin
      BoxSelectFace := fTop;
      BoxOffset := 0.35;
      BoxAngle := 0;
      BoxUStep := 0.2;
      BoxRasterPaint(BoxSelectFace, BoxOffset, BoxAngle, BoxUStep);

    end;
  end else if controlMode = cmManual then begin
    SetRCValue(2,8,'Manual');
    SetRCBackColor(2, 9, $7F00FFFF);
    SetRCBackColor(2,10, $7FFFFFFF);
    SetRCBackColor(3, 8, $7FAAAAAA);
    case lr_mode of
      1: begin
          if KeyPressed(vk_left) then begin
            sg_x := sg_x-0.05;
          end else if KeyPressed(vk_right) then begin
            sg_x := sg_x+0.05;
          end
         end;
      3: begin
          if KeyPressed(vk_left) then begin
            sg_y := sg_y-0.05;
          end else if KeyPressed(vk_right) then begin
            sg_y := sg_y+0.05;
          end
         end;
      5: begin
          if KeyPressed(vk_left) then begin
            sg_z := sg_z-0.05;
          end else if KeyPressed(vk_right) then begin
            sg_z := sg_z+0.05;
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
            sg_x := sg_x-0.05;
          end else if KeyPressed(vk_up) then begin
            sg_x := sg_x+0.05;
          end
         end;
      4: begin
          if KeyPressed(vk_down) then begin
            sg_y := sg_y-0.05;
          end else if KeyPressed(vk_up) then begin
            sg_y := sg_y+0.05;
          end
         end;
      6: begin
          if KeyPressed(vk_down) then begin
            sg_z := sg_z-0.05;
          end else if KeyPressed(vk_up) then begin
            sg_z := sg_z+0.05;
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

  SetRobotPos(2, sg_x, sg_y, sg_z, sg_theta);

  //end jm

  UpdateScrew(1);

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


  tis := tis + 0.04;

  SetRCValue(3, 7, state);
end;


procedure Initialize;
var i: integer;
begin


  irobot := 0;
  iScrew := 1;

  iB5 := GetSolidIndex(irobot, 'B5');
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

  state := 'idle';
  Tol := 0.2;

  //JM
  lr_mode:=1;
  ud_mode:= 4;
  //spray gun
  sg_x:=0;
  sg_y:=-0.5;
  sg_z:=0.6;
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
end;
