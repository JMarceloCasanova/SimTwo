const
 NumJoints = 6;
type
  faces = (fTop, fBottom, fLeft, fRight, fBack, fFront);
  controlModes = (cmManual, cmPaintModes, cmUDPServer, cmNone);
  paintModes = (pmNone, pmBoxRaster, pmLoadedTraj);

  TTrajectory = record
    points : array [0..500] of TPoint3D;
    direction : array [0..500] of TPoint3D;
    count: integer;
    nextPoint: integer;
    nextpointPerone: double;
  end;
// Global Variables
var

  irobot, iwrist, itool: integer;
  A, B, C: double;
  robBase: TPoint3D;
  
  R01, R12, R23: Matrix;
  R03: Matrix;

  JointPos: array[0..NumJoints - 1] of double;

  ReqThetas: matrix;
  Tol: double;

  //JM
  script_period: double;
  lr_mode: integer;
  ud_mode: integer;

  controlMode: controlModes;
  paintMode: paintModes;

  ext: TExtents;
  trajs : array [0..1] of TTrajectory;
  //spray gun
  //sg_x, sg_y, sg_z,
  sg : TPoint3D;
  sg_rot: Matrix;
  sg_theta: double;

  connection: Boolean;
  await_synack: Boolean;

  receiveResultColors: Boolean;
  receiveResultColorsSize, receiveResultColorsPos: Integer;

  colors: array of TPoint3D;

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
function subTPoint3D(a, b:TPoint3D):TPoint3D;
begin
  result.x := a.x - b.x;
  result.y := a.y - b.y;
  result.z := a.z - b.z;
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
function TPoint3DtoMat(A:TPoint3D): Matrix;
begin
  result := Mzeros(3,1);
  MSetV(result, 0,0,A.X);
  MSetV(result, 1,0,A.Y);
  MSetV(result, 2,0,A.Z);
end;
function MattoTPoint3D(A: Matrix): TPoint3D;
begin
  result.X := MGetV(A, 0,0);
  result.Y := MGetV(A, 1,0);
  result.Z := MGetV(A, 2,0);
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


// Pace here the Inverse Kinematics calculations
function IK(toolPos: TPoint3D; toolRot: matrix): matrix;
var D: double; //A upperArm B elbow_offset C elbow_and_lowerArm
    theta1, theta2, theta3, theta4, theta5, theta6: double;
    beta: double;
    Ow : TPoint3D;//Wrist center
    configuration: Integer;
    toolDist: TPoint3D;
    R03, R36: matrix;
begin
  toolPos := subTPoint3D(toolPos, robBase);

  toolDist.X := 0.265168;
  toolDist.Y := -0.097242;
  toolDist.Z := -0.020563;
  Ow := MattoTPoint3D( Msub(TPoint3DtoMat(toolPos),  MMult(toolRot, TPoint3DtoMat(toolDist))) );

  configuration := 1; //upper
  //configuration := -1; //lower
  D := sqrt(B*B+C*C);
  theta1 := atan2(Ow.Y, Ow.X);
  beta := atan2(C,B);
  theta3 := configuration*( beta - arccos( (Ow.X*Ow.X+Ow.Y*Ow.Y + Ow.Z*Ow.Z - A*A-D*D) / (2*A*D) ) );
  theta2 := atan2(Ow.Y, Ow.X) + configuration*atan2(D*sin(beta-theta3), A-D*cos(beta-theta3)) -PI/2;

  R03 := Meye(3);
  MsetV(R03, 0, 0, cos(theta1)*cos(theta2+theta3));
  MsetV(R03, 0, 1, sin(theta1)*cos(theta2+theta3));
  MsetV(R03, 0, 2, sin(theta2+theta3));
  MsetV(R03, 1, 0, sin(theta1));
  MsetV(R03, 1, 1, -cos(theta1));
  MsetV(R03, 1, 2, 0);
  MsetV(R03, 2, 0, cos(theta1)*sin(theta2+theta3));
  MsetV(R03, 2, 1, sin(theta1)*sin(theta2+theta3));
  MsetV(R03, 2, 2, cos(theta2+theta3));

  R36 := MMult(Mtran(R03), toolRot);
  result := Mzeros(6, 1);
  MSetV(result, 0, 0, theta1);
  MSetV(result, 1, 0, theta2);
  MSetV(result, 2, 0, theta3);
end;



procedure SetThetas(Thetas: matrix);
var i: integer;
begin
  for i := 0 to 5 do begin
    SetAxisPosRef(iRobot, i, Mgetv(Thetas, i, 0));
  end;
  WriteLn('setThetas');
  MatrixToRange(11, 3, Thetas);
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


procedure ManualControl(velocity: double);
var delta_v: double;
begin
  delta_v := velocity*script_period;

  if RCButtonPressed(10, 11) then begin//reset
    sg_rot := Meye(3);
    Msetv(sg_rot, 0, 0, 0);
    Msetv(sg_rot, 2, 0, -1);
    Msetv(sg_rot, 2, 2, 0);
    Msetv(sg_rot, 0, 2, 1); 
    MatrixToRange(10, 12, sg_rot); 
  end;
  if RCButtonPressed(5, 14) then begin
    sg_rot := RangeToMatrix(10, 12, 3, 3);
  end;
  
  
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
          sg.x := sg.x-delta_v;
        end else if KeyPressed(vk_right) then begin
          sg.x := sg.x+delta_v;
        end
      end;
    3: begin
        if KeyPressed(vk_left) then begin
          sg.y := sg.y-delta_v;
        end else if KeyPressed(vk_right) then begin
          sg.y := sg.y+delta_v;
        end
      end;
    5: begin
        if KeyPressed(vk_left) then begin
          sg.z := sg.z-delta_v;
        end else if KeyPressed(vk_right) then begin
          sg.z := sg.z+delta_v;
        end
      end;
    7: begin
        if KeyPressed(vk_left) then begin
          sg_theta := sg_theta+delta_v;
        end else if KeyPressed(vk_right) then begin
          sg_theta := sg_theta-delta_v;
        end
      end;
  end;
  case ud_mode of
    2: begin
        if KeyPressed(vk_down) then begin
          sg.x := sg.x-delta_v;
        end else if KeyPressed(vk_up) then begin
          sg.x := sg.x+delta_v;
        end
      end;
    4: begin
        if KeyPressed(vk_down) then begin
          sg.y := sg.y-delta_v;
        end else if KeyPressed(vk_up) then begin
          sg.y := sg.y+delta_v;
        end
      end;
    6: begin
        if KeyPressed(vk_down) then begin
          sg.z := sg.z-delta_v;
        end else if KeyPressed(vk_up) then begin
          sg.z := sg.z+delta_v;
        end
      end;
    8: begin
        if KeyPressed(vk_down) then begin
          sg_theta := sg_theta-delta_v;
        end else if KeyPressed(vk_up) then begin
          sg_theta := sg_theta+delta_v;
        end
      end;
    end;

end;

procedure sendUdp(msg: string);
begin
  WriteUDPData(GetRCText(1,16), strtoint(GetRCText(2,16)), msg);
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

procedure DecodeCommands(var command: string);//Client POV
var StrPacketRec, StrPacketSend: TStringList;
    Triangles: TTriangles;
    i, j, l: integer;
    st, en, tries:integer;
begin
  StrPacketRec := TStringList.create;
  StrPacketSend := TStringList.create;

  StrPacketRec.Delimiter := ' ';
  StrPacketRec.DelimitedText := command;

  tries := 100000;
  while (StrPacketRec.Count > 0) and (tries>0) do begin
    
    i := StrPacketRec.indexof('Disconnect');
    if (i >= 0) and (i < StrPacketRec.count) then begin
      connection := False;
      WriteLn('Disconnected');
      StrPacketRec.Delete(i);
    end;
    i := StrPacketRec.indexof('ReadTrianglesCount');
    if (i >= 0) and (i < StrPacketRec.count) then begin
      Triangles := GetPaintTargetTriangles(0);
      EncodeInteger(StrPacketSend, 'a', length(Triangles));
      sendUdp(StrPacketSend.text);
      WriteLn('sent');
      StrPacketRec.Delete(i);
    end else begin
      i := StrPacketRec.indexof('Read80Triangles');
      if (i >= 0) and (i < StrPacketRec.count) then begin
        st := strtoint(StrPacketRec[i+1]);
        Triangles := GetPaintTargetTriangles(0);
        if length(Triangles) > 0 then begin
          if length(Triangles)-80 < st then en := length(Triangles)
          else en := st+80;
          Writeln('Sending');
          Writeln(inttostr(en-st));
          Writeln('triangles');
          //for j:=0 to length(Triangles)-1 do begin
          for j:=st to en-1 do begin
            //WriteLn(inttostr(j));
            EncodeInteger(StrPacketSend, 'b', j);
            EncodeDouble(StrPacketSend, 'b', Triangles[j].vertice0.X);
            EncodeDouble(StrPacketSend, 'b', Triangles[j].vertice0.Y);
            EncodeDouble(StrPacketSend, 'b', Triangles[j].vertice0.Z);
            EncodeDouble(StrPacketSend, 'b', Triangles[j].vertice1.X);
            EncodeDouble(StrPacketSend, 'b', Triangles[j].vertice1.Y);
            EncodeDouble(StrPacketSend, 'b', Triangles[j].vertice1.Z);
            EncodeDouble(StrPacketSend, 'b', Triangles[j].vertice2.X);
            EncodeDouble(StrPacketSend, 'b', Triangles[j].vertice2.Y);
            EncodeDouble(StrPacketSend, 'b', Triangles[j].vertice2.Z);
            EncodeDouble(StrPacketSend, 'b', Triangles[j].normal.X);
            EncodeDouble(StrPacketSend, 'b', Triangles[j].normal.Y);
            EncodeDouble(StrPacketSend, 'b', Triangles[j].normal.Z);
            EncodeDouble(StrPacketSend, 'b', Triangles[j].center.X);
            EncodeDouble(StrPacketSend, 'b', Triangles[j].center.Y);
            EncodeDouble(StrPacketSend, 'b', Triangles[j].center.Z);
            EncodeDouble(StrPacketSend, 'b', Triangles[j].area);
            
            EncodeInteger(StrPacketSend, 'b', length(Triangles[j].neighbors));
            for l:=0 to length(Triangles[j].neighbors) - 1 do begin
              //WriteLn(inttostr(Triangles[j].neighbors[l]));
              EncodeInteger(StrPacketSend, 'b', Triangles[j].neighbors[l]);
            end;
            EncodeDouble(StrPacketSend, 'b', 0);
            if ((j mod 25) = 0) then WriteLn(StrPacketSend.text);
          end;

        end else begin              
          WriteLn('no triangles');
          EncodeDouble(StrPacketSend, 'b', 0);  
        end;
        sendUdp(StrPacketSend.text);
        WriteLn(StrPacketSend.text);
        WriteLn('sent');
        StrPacketRec.Delete(i);
        //StrPacketRec.Delete(i+1);
      end;  
    end;
    i := StrPacketRec.indexof('WriteCountResultColors');
    if (i >= 0) and (i < StrPacketRec.count) then begin
      en := strtoint(StrPacketRec[i+1]);
      WriteLn('WriteCountResultColors');
      setLength(colors, en);
      receiveResultColorsSize := en;
      StrPacketRec.Delete(i);
      //StrPacketRec.Delete(i+1);
    end;
    i := StrPacketRec.indexof('WriteResultColors');
    if (i >= 0) and (i < StrPacketRec.count) then begin
      WriteLn('ResultColors');
      receiveResultColors := True;
      StrPacketRec.Delete(i);
    end;
    i := StrPacketRec.indexof('WriteResultColor');
    if receiveResultColors and (i >= 0) and (i < StrPacketRec.count) then begin
      WriteLn(inttostr(receiveResultColorsPos));
      if (receiveResultColorsPos+1) = receiveResultColorsSize then begin
        receiveResultColorsPos := 0;
        receiveResultColorsSize := 0;
        receiveResultColors := False;
        WriteLn('SetResultTrianglesColor length(colors):');
        WriteLn(inttostr(length(colors)));
        SetResultTrianglesColor(0, colors);
      end else begin
        //WriteLn(StrPacketRec[i+1]);
        if (receiveResultColorsPos mod 3) = 0 then colors[receiveResultColorsPos div 3].X := strtofloat(StrPacketRec[i+1]);
        if ((receiveResultColorsPos+1) mod 3) = 0 then colors[receiveResultColorsPos div 3].Y := strtofloat(StrPacketRec[i+1]);
        if ((receiveResultColorsPos+2) mod 3) = 0 then colors[receiveResultColorsPos div 3].Z := strtofloat(StrPacketRec[i+1]);
        receiveResultColorsPos := receiveResultColorsPos + 1;
      end;
      StrPacketRec.Delete(i);
      //StrPacketRec.Delete(i+1);
    end;
    tries := tries-1;
  end;
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
      while true do begin
        StrPacket.text := ReadUDPData();
        txt := StrPacket.text;
        if txt = '' then break;
        WriteLn(txt);
        DecodeCommands(txt);
        // Read Motor Speed Reference
        a := DecodeDoubleDef(StrPacket, 'a', 0);
      end;
      //EncodeInteger(StrPacket,'Enc1', GetAxisOdo(0, 0));
      for i := 0 to 3 do begin
        //EncodeDouble(StrPacket, 's' + inttostr(i), GetSensorVal(0, i));
      end;
      //WriteUDPData(GetRCText(1,16), strtoint(GetRCText(2,16)), StrPacket.text);
      
    finally
      StrPacket.free;
    end;  
  end else begin
    while true do begin
        txt := ReadUDPData();
        if txt = '' then break;
        WriteLn(txt);
        if await_synack and (txt = 'SYNACK') then begin
          connection := true;
          SetRCValue(4, 15, 'done');
          WriteLn('connected');
        end else if txt = 'SYN' then begin
          sendUdp('ACK');
          await_synack := True;
          SetRCValue(4, 15, 'await_synack');
        end;
      end;
  end;

end;

procedure RunTrajectory(vel: double; trajIndex: integer);
var dirnorm: TPoint3D;
    delta_v: double;
begin
  SetRCValue(1,1,IntToStr(trajs[trajIndex].nextPoint));
  delta_v := vel*script_period;
  if trajs[trajIndex].nextPoint = 0 then begin
    trajs[trajIndex].nextPoint := 1;
    trajs[trajIndex].nextpointPerone := 0;
    sg := trajs[trajIndex].points[0];
  end else if trajs[trajIndex].nextPoint > trajs[trajIndex].count then begin
    paintMode := pmNone;
  end;

  if (trajs[trajIndex].nextpointPerone>0.98) then begin
    while ((trajs[trajIndex].points[trajs[trajIndex].nextPoint].z<0.01) and (trajs[trajIndex].nextPoint < trajs[trajIndex].count)) do begin
      trajs[trajIndex].nextPoint := trajs[trajIndex].nextPoint+1;
    end;
    trajs[trajIndex].nextPoint := trajs[trajIndex].nextPoint+1;
    trajs[trajIndex].nextpointPerone := 0;
  end;

  dirnorm := normTPoint3D(diffTPoint3D(trajs[trajIndex].points[trajs[trajIndex].nextPoint], trajs[trajIndex].points[trajs[trajIndex].nextPoint-1]));
  sg := AddTPoint3D(sg, scaleTPoint3D(dirnorm, delta_v));
  sg_rot := Meye(3);
  Msetv(sg_rot, 0, 0, trajs[trajIndex].direction[trajs[trajIndex].nextPoint].x);//a1
  Msetv(sg_rot, 1, 0, trajs[trajIndex].direction[trajs[trajIndex].nextPoint].y);//a2
  Msetv(sg_rot, 2, 0, trajs[trajIndex].direction[trajs[trajIndex].nextPoint].z);//a3
  Msetv(sg_rot, 0, 1, -trajs[trajIndex].direction[trajs[trajIndex].nextPoint].y);//b1
  Msetv(sg_rot, 1, 1, trajs[trajIndex].direction[trajs[trajIndex].nextPoint].x);//b2
  Msetv(sg_rot, 2, 1, 0);//b3
  Msetv(sg_rot, 0, 2, -trajs[trajIndex].direction[trajs[trajIndex].nextPoint].z*trajs[trajIndex].direction[trajs[trajIndex].nextPoint].x);//a2b3-a3b2
  Msetv(sg_rot, 1, 2, trajs[trajIndex].direction[trajs[trajIndex].nextPoint].z*-trajs[trajIndex].direction[trajs[trajIndex].nextPoint].y);//a3b1-a1b3
  Msetv(sg_rot, 2, 2, trajs[trajIndex].direction[trajs[trajIndex].nextPoint].x*trajs[trajIndex].direction[trajs[trajIndex].nextPoint].x-trajs[trajIndex].direction[trajs[trajIndex].nextPoint].y*-trajs[trajIndex].direction[trajs[trajIndex].nextPoint].y);//a1b2-a2b1

  trajs[trajIndex].nextpointPerone := distTPoint3D(sg, trajs[trajIndex].points[trajs[trajIndex].nextPoint-1])/distTPoint3D(trajs[trajIndex].points[trajs[trajIndex].nextPoint], trajs[trajIndex].points[trajs[trajIndex].nextPoint-1]);

end;

procedure DrawTrajectory(trajIndex: integer);
var i: integer;
begin
  ClearTrail(0);
  SetTrailColor(0, 0, 255, 0);
  for i:= 0 to (trajs[trajIndex].count-1) do begin
    AddTrailNode(0, trajs[trajIndex].points[i].X, trajs[trajIndex].points[i].Y, trajs[trajIndex].points[i].z);
    if i<10 then begin
      SetRCValue(2,21+i, '[go]');
      SetRCValue(4,21+i, FloatToStr(trajs[trajIndex].points[i].Y));
      SetRCValue(3,21+i, FloatToStr(trajs[trajIndex].points[i].X));
      SetRCValue(5,21+i, FloatToStr(trajs[trajIndex].points[i].Z));
    end;
  end;
  SetRCValue(1, 22, IntToStr(trajs[trajIndex].count));
  SetRCValue(3,21, FloatToStr(trajs[trajIndex].points[0].X));
  SetRCValue(4,21, FloatToStr(trajs[trajIndex].points[0].Y));
  SetRCValue(5,21, FloatToStr(trajs[trajIndex].points[0].Z));
  if trajs[trajIndex].points[0].Z > 0.01 then 
    sg := trajs[trajIndex].points[0];
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
  for i:= 0 to traj.count-1 do begin//Should be (traj.count div 4) ???
    traj.points[i*4].X := faceExt.min.X + i*2*BoxUStep;
    traj.points[i*4].Y := faceExt.min.Y;
    traj.points[i*4].Z := faceExt.max.Z;
    traj.direction[i*4].X := 0;
    traj.direction[i*4].Y := 0;
    traj.direction[i*4].Z := -1;
    traj.points[i*4+1].X := faceExt.min.X + i*2*BoxUStep;
    traj.points[i*4+1].Y := faceExt.max.Y;
    traj.points[i*4+1].Z := faceExt.max.Z;
    traj.direction[i*4+1].X := 0;
    traj.direction[i*4+1].Y := 0;
    traj.direction[i*4+1].Z := -1;
    traj.points[i*4+2].X := faceExt.min.X + (i*2+1)*BoxUStep;
    traj.points[i*4+2].Y := faceExt.max.Y;
    traj.points[i*4+2].Z := faceExt.max.Z;
    traj.direction[i*4+2].X := 0;
    traj.direction[i*4+2].Y := 0;
    traj.direction[i*4+2].Z := -1;
    traj.points[i*4+3].X := faceExt.min.X + (i*2+1)*BoxUStep;
    traj.points[i*4+3].Y := faceExt.min.Y;
    traj.points[i*4+3].Z := faceExt.max.Z;
    traj.direction[i*4+3].X := 0;
    traj.direction[i*4+3].Y := 0;
    traj.direction[i*4+3].Z := -1;
  end;
  result := traj;
end;


procedure Control;
var i: integer;
    wristPos, toolPos: Matrix;
    toolRot: Matrix;
    BoxSelectFace: faces;
    BoxOffset: double;
    BoxUStep, BoxVExtend: double;

    loadMat: Matrix;
    target_pos:TPoint3D;
begin
  if RCButtonPressed(6, 4) then ResetPaintTargetPaint(0);
  if RCButtonPressed(7, 4) then SetPaintTargetPaintMode(0, pmPaint);
  if RCButtonPressed(8, 4) then SetPaintTargetPaintMode(0, pmHeatmap);
  if RCButtonPressed(9, 4) then SetPaintTargetPaintMode(0, pmResult);
  if RCButtonPressed(10, 4) then SetSprayGunOn(0);
  if RCButtonPressed(11, 4) then SetSprayGunOff(0);

  if RCButtonPressed(8, 15) then begin
    if GetRCText(8,16) <> '' then begin
      loadMat := MLoad(GetRCText(8,16));
      //MatrixToRange(8, 16, loadMat);
      setLength(colors, MNumRows(loadMat));
      for i:=0 to MNumRows(loadMat)-1 do begin
        colors[i].X := MGetV(loadMat, i, 0);
        colors[i].Y := MGetV(loadMat, i, 1);
        colors[i].Z := MGetV(loadMat, i, 2);
      end;
      SetResultTrianglesColor(0, colors);
    end;
  end;

  if RCButtonPressed(9, 15) then begin
    if GetRCText(9,16) <> '' then begin
      loadMat := MLoad(GetRCText(9,16));
      //MatrixToRange(9, 16, loadMat);
      ext.Min.x := MGetV(loadMat, 0, 0);
      ext.Min.y := MGetV(loadMat, 0, 1);
      ext.Min.z := MGetV(loadMat, 0, 2);
      ext.Max.x := MGetV(loadMat, 1, 0);
      ext.Max.y := MGetV(loadMat, 1, 1);
      ext.Max.z := MGetV(loadMat, 1, 2);
    end;
  end;

  if RCButtonPressed(10, 15) then begin
    if GetRCText(10,16) <> '' then begin
      loadMat := MLoad(GetRCText(10,16));
      target_pos := GetThingPos(0);
      //MatrixToRange(11, 18, loadMat);
      trajs[1].count := MNumRows(loadMat);
      for i:=0 to MNumRows(loadMat)-1 do begin
        trajs[1].points[i].X := MGetV(loadMat, i, 0);
        trajs[1].points[i].Y := MGetV(loadMat, i, 1);
        trajs[1].points[i].Z := MGetV(loadMat, i, 2);
        trajs[1].points[i] := addTPoint3D(target_pos, trajs[1].points[i]);
        trajs[1].direction[i].X := MGetV(loadMat, i, 3);
        trajs[1].direction[i].Y := MGetV(loadMat, i, 4);
        trajs[1].direction[i].Z := MGetV(loadMat, i, 5);
      end;
      DrawTrajectory(1);
    end;
  end;

  if RCButtonPressed(6, 13) then begin
    sg := ext.Max;
  end;
  if RCButtonPressed(7, 13) then begin
    sg := ext.Min;
  end;

  if RCButtonPressed(1,9) then controlMode := cmManual;
  if RCButtonPressed(1,10) then controlMode := cmPaintModes;
  if RCButtonPressed(1,11) then controlMode := cmUDPServer;
  if RCButtonPressed(1,12) then controlMode := cmNone;

  if RCButtonPressed(4, 16) then connection := False;

  if controlMode = cmPaintModes then begin
    SetRCValue(2,8,'PaintModes');
    SetRCBackColor(2, 10, $0000FF00);
    SetRCBackColor(2, 9, $7FFFFFFF);
    SetRCBackColor(2,11, $7FFFFFFF);
    SetRCBackColor(3, 8, $7FFFFFFF);
    if RCButtonPressed(3,9) then begin
      paintMode := pmBoxRaster;
      BoxSelectFace := fTop;
      BoxOffset := 0.5;
      BoxUStep := 0.1;
      BoxVExtend := 0.3;
      trajs[0] := CalculateBoxRaster(BoxSelectFace, BoxOffset, BoxUStep, BoxVExtend);
      DrawTrajectory(0);
    end else if RCButtonPressed(3,10) then begin
      paintMode := pmLoadedTraj;
      DrawTrajectory(1);
    end else if RCButtonPressed(3,11) then begin
      paintMode := pmNone;
      ClearTrail(0);
      SetRCValue(4, 8, 'None');
    end;
    if paintMode = pmBoxRaster then begin
      SetRCValue(4, 8, 'Box Raster');
      //WriteLn('e');
      //RunTrajectory(0.015, trajs[0]);
      RunTrajectory(0.3, 0);
    end else if paintMode=pmLoadedTraj then begin
      SetRCValue(4, 8, 'Loaded Traj');
      for i:=0 to trajs[1].count-1 do begin
        if (i<10) then begin
          if RCButtonPressed(2,21+i) then begin
            sg := trajs[1].points[i];
          end;
        end;
      end;
      //WriteLn('e');
      //RunTrajectory(0.015, traj2);
      RunTrajectory(0.3, 1);
    end;
    for i:=0 to trajs[0].count-1 do begin
      if (i<10) then begin
        if RCButtonPressed(2,21+i) then begin
          sg := trajs[0].points[i];
        end;
      end;
    end;
  end else if controlMode = cmManual then begin
    SetRCValue(2,8,'Manual');
    SetRCBackColor(2, 9, $7F00FFFF);
    SetRCBackColor(2,10, $7FFFFFFF);
    SetRCBackColor(2,11, $7FFFFFFF);
    SetRCBackColor(3, 8, $7FAAAAAA);
    ManualControl(0.1);
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
  end else if controlMode = cmNone then begin
    SetRCValue(2,8,'None');
  end;

  if controlMode <> cmNone then begin
    //SetRobotPos(0, sg.x, sg.y, sg.z, sg_theta);
    SetSolidPos(0, 0, sg.x, sg.y, sg.z);
    SetSolidRotationMat(0,0, sg_rot);
  end;


  if RCButtonPressed(14, 8) then begin
    SetRCValue(15, 8, 'AvgSprayThickness');
    SetRCValue(16, 8, format('%.6f',[CalculateAvgSprayThickness(0)]));
    SetRCValue(15, 9, 'SDSprayThickness');
    SetRCValue(16, 9, format('%.6f',[CalculateSDSprayThickness(0)]));
    SetRCValue(15, 10, 'PosAvgSprayThickness');
    SetRCValue(16, 10, format('%.6f',[CalculatePosAvgSprayThickness(0)]));
    SetRCValue(15, 11, 'PosSDSprayThickness');
    SetRCValue(16, 11, format('%.6f',[CalculatePosSDSprayThickness(0)]));
    SetRCValue(15, 12, 'PosMinSprayThickness');
    SetRCValue(16, 12, format('%.6f',[CalculatePosMinSprayThickness(0)]));
    SetRCValue(15, 13, 'MinSprayThickness');
    SetRCValue(16, 13, format('%.6f',[CalculateMinSprayThickness(0)]));
    SetRCValue(15, 14, 'MaxSprayThickness');
    SetRCValue(16, 14, format('%.6f',[CalculateMaxSprayThickness(0)]));
    SetRCValue(15, 15, 'SprayCoverage');
    SetRCValue(16, 15, format('%.6f',[CalculateSprayCoverage(0)]));
  end;

  //Robot Control
  
  wristPos := GetSolidPosMat(iRobot, iwrist);
  MatrixToRange(11, 2, wristPos);

  toolPos := GetSolidPosMat(iRobot, itool);
  MatrixToRange(16, 2, toolPos);

  toolrot := GetSolidRotMat(iRobot, itool);
  MatrixToRangeF(16, 4, toolRot, '%.3f');

  // Read joint positions
  for i := 0 to NumJoints -1 do begin
    JointPos[i] := GetAxisPos(irobot, i);
  end;

  // and show
  for i := 0 to NumJoints -1 do begin
    SetRCValue(3 + i, 2, format('%.3g',[Deg(JointPos[i])]));
  end;

  // control equations
  // ...

  if RCButtonPressed(10, 1) then begin
    ReqThetas := RangeToMatrix(11,1,3,1);
    ReqThetas := Ik(MattoTPoint3D(ReqThetas), Meye(3));
  end;
  SetThetas(ReqThetas);


end;


procedure Initialize;
var i: integer;
begin
  irobot := 1;

  iwrist := GetSolidIndex(irobot, 'wrist');
  itool := GetSolidIndex(irobot, 'tool');

  SetRCValue(10, 1, '[Ik(Pos)]');
  SetRCValue(10, 2, 'wristPos');
  SetRCValue(15, 2, 'toolPos');
  SetRCValue(15, 4, 'toolRot');
  SetRCValue(2, 1, 'Joint');
  SetRCValue(2, 2, 'Pos (deg)');
  for i := 0 to NumJoints -1 do begin
    SetRCValue(3 + i, 1, format('%d',[i]));
  end;

  robBase.X := -2 + 0.08685;
  robBase.Y := 0 + 0.0025;
  robBase.Z := 0 + 0.1560025 + 0.312005/2;
  A := 1.445008;
  B := 0.560;
  C := 0.653 + 0.784 + 0.255/2 -0.0362;
  Tol := 0.2;

  ReqThetas := Mzeros(6, 1);

  //JM
  script_period := ScriptPeriod();
  lr_mode:=1;
  ud_mode:= 4;
  //spray gun
  sg.x:=0;
  sg.y:=-1.5;
  sg.z:=0.8;
  sg_theta:=0;
  sg_rot := Meye(3);
  Msetv(sg_rot, 0, 0, 0);
  Msetv(sg_rot, 2, 0, -1);
  Msetv(sg_rot, 2, 2, 0);
  Msetv(sg_rot, 0, 2, 1);
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
  receiveResultColors := False;
  receiveResultColorsSize := 0;
  receiveResultColorsPos := 0;
end;
