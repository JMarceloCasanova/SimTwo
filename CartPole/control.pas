// Global Variables Here
var 
xref: double;
robIndex, robBaseIndex, robPendIndex: integer;
robBaseOrigin, robPendOrigin: TPoint3D;
robBaseOriginRot, robPendOriginRot: Matrix;
resetTimes: integer;


procedure EncodeDouble(StrPacket: TStringList; name: string; data: double);
begin
  StrPacket.add(name);
  StrPacket.add(format('%.6g',[data]));
  StrPacket.add('');
end;

function DecodeDoubleDef(StrPacket: TStringList; name: string; defval: double): double;
var i: integer;
begin
  result := defval;            
  WriteLn(StrPacket.text);
  i := StrPacket.indexof(name);
  if (i < 0) or (i + 1 >= StrPacket.count) then exit;
  result := strtofloat(StrPacket[i+1]);
end;

procedure ResetScene();
begin
  SetSolidPos(robIndex, robBaseIndex, robBaseOrigin.x, robBaseOrigin.y, robBaseOrigin.z);
  SetSolidPos(robIndex, robPendIndex, robPendOrigin.x, robPendOrigin.y, robPendOrigin.z);
  SetSolidRotationMat(robIndex, robBaseIndex, robBaseOriginRot);
  SetSolidRotationMat(robIndex, robPendIndex, robPendOriginRot);

  SetSolidAngularVel(robIndex, robBaseIndex, 0, 0, 0);
  SetSolidLinearVel(robIndex, robBaseIndex, 0, 0, 0);
  SetSolidAngularVel(robIndex, robPendIndex, 0, 0, 0);
  SetSolidLinearVel(robIndex, robPendIndex, 0, 0, 0);
  WriteLn('Reseted Scene');
end;

// this procedure is called periodicaly (default: 40 ms)
procedure Control;
var StrPacket: TStringList;
    x, vx, teta, w, U: double;
    i: Integer;
begin
  teta := GetAxisPos(0, 0);
  w := GetAxisSpeed(0, 0);
  x := GetAxisPos(0, 1);
  vx := GetAxisSpeed(0, 1);
  
  StrPacket := TStringList.create;
  try
    //(x, x_dot, theta, theta_dot)
    EncodeDouble(StrPacket,'a', GetAxisPos(0, 1));
    EncodeDouble(StrPacket,'b', GetAxisSpeed(0, 1));
    EncodeDouble(StrPacket,'c', GetAxisPos(0, 0));
    EncodeDouble(StrPacket,'d', GetAxisSpeed(0, 0));

    WriteUDPData('127.0.0.1', 9810, StrPacket.text);
    //WriteLn(StrPacket.text);

    StrPacket.text := ReadUDPData();
    if StrPacket.text <> '' then begin
      WriteLn(StrPacket.text);
      // Read control
      i := StrPacket.indexof('reset');
      WriteLn(intToStr(i));
      if ((i >= 0)) or RCButtonPressed(1,1) then begin
        WriteLn('HEREEEEEEEEEEE');
        resetTimes := 3;
      end;

      U := DecodeDoubleDef(StrPacket, 'u', 0);
      if U <> 0 then WriteLn(floatToStr(U));
    end;  

    if RCButtonPressed(1,1) then resetTimes := 3;
    
    //SetAxisVoltageRef(irobot, 0, U);


    SetRCValue(2, 2, format('%.2g',[U]));

  finally
    StrPacket.free;
  end;

  If resetTimes>0 then begin
    resetTimes := resetTimes - 1;
    ResetScene();
  end else begin
    SetAxisTorqueRef(0, 1, U);
  end;
end;




// this procedure is called once when the script is started
procedure Initialize;
begin
  robIndex := GetRobotIndex('InvPend');
  robBaseIndex := GetSolidIndex(robIndex, 'Base');
  robPendIndex := GetSolidIndex(robIndex, 'Pend');
  robBaseOrigin := GetSolidPos(robIndex, robBaseIndex);
  robPendOrigin := GetSolidPos(robIndex, robPendIndex);
  robBaseOriginRot := GetSolidRotMat(robIndex, robBaseIndex);
  robPendOriginRot := GetSolidRotMat(robIndex, robPendIndex);

  xref := 0;
  resetTimes := 0;
end;
