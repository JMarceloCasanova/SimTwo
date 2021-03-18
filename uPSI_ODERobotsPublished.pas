unit uPSI_ODERobotsPublished;
{
This file has been generated by UnitParser v0.7, written by M. Knight
and updated by NP. v/d Spek and George Birbilis.
Source Code from Carlo Kok has been used to implement various sections of
UnitParser. Components of ROPS are used in the construction of UnitParser,
code implementing the class wrapper is taken from Carlo Kok's conv utility

}
interface

uses
   SysUtils
  ,Classes
  ,uPSComponent
  ,uPSRuntime
  ,uPSCompiler
  ;

type
(*----------------------------------------------------------------------------*)
  TPSImport_ODERobotsPublished = class(TPSPlugin)
  protected
    procedure CompileImport1(CompExec: TPSScript); override;
    procedure ExecImport1(CompExec: TPSScript; const ri: TPSRuntimeClassImporter); override;
  end;


{ compile-time registration functions }
procedure SIRegister_ODERobotsPublished(CL: TPSPascalCompiler);

{ run-time registration functions }
procedure RIRegister_ODERobotsPublished_Routines(S: TPSExec);

procedure Register;

implementation


uses
   Graphics
  ,Types
  ,ODERobots
  ,PathFinder
  ,dynmatrix
  ,GLKeyboard
  ,ODEGL
  ,GLVectorFileObjects
  ,GLGeometryBB
  ,ODERobotsPublished
  ;


procedure Register;
begin
  RegisterComponents('Pascal Script', [TPSImport_ODERobotsPublished]);
end;

(* === compile-time registration functions === *)
(*----------------------------------------------------------------------------*)
procedure SIRegister_ODERobotsPublished(CL: TPSPascalCompiler);
begin
  CL.AddTypeS('TAxisPoint', 'record pos : double; speed : double; final_time : '
   +'double; end');
  CL.AddTypeS('TAxisState', 'record pos : double; vel : double; Torque : double'
   +'; Vm : double; Im : double; end');
  CL.AddTypeS('TState2D', 'record x : double; y : double; angle : double; end');
  CL.AddTypeS('TPoint3D', 'record x : double; y : double; z : double; end');
  CL.AddTypeS('TSpringDef', 'record K : double; ZeroPos : double; end');
  CL.AddTypeS('TMotorPars', 'record Ri : double; Li : double; Ki : double; Vmax'
   +' : double; Imax : double; GearRatio : double; simple : boolean; active : b'
   +'oolean; end');
  CL.AddTypeS('TMotorControllerPars', 'record Ki : double; Kd : double; Kp : do'
   +'uble; Kf : double; end');
  CL.AddTypeS('TFrictionDef', 'record Bv : double; Fc : double; CoulombLimit : '
   +'double; end');
  CL.AddTypeS('TRGBAColor', 'record Red : integer; Green : integer; Blue : inte'
   +'ger; alpha : integer; end');
  CL.AddTypeS('TExtents', 'record Min : TPoint3D; Max : TPoint3D; end');
 CL.AddDelphiFunction('Procedure SetFireScale( x, y, z : double)');
 CL.AddDelphiFunction('Procedure SetFirePosition( x, y, z : double)');
 CL.AddDelphiFunction('Procedure StartFire');
 CL.AddDelphiFunction('Procedure StopFire');
 CL.AddDelphiFunction('Procedure StopSolidFire( R, I : integer)');
 CL.AddDelphiFunction('Procedure StartSolidFire( R, I : integer)');
 CL.AddDelphiFunction('Function GetSceneConstant( constantName : string; defaultValue : double) : double');
 CL.AddDelphiFunction('Procedure SetRobotPos( R : integer; x, y, z, teta : double)');
 CL.AddDelphiFunction('Function GetRobotIndex( ID : string) : integer');
 CL.AddDelphiFunction('Function GetRobotPos2D( R : integer) : TState2D');
 CL.AddDelphiFunction('Function GetRobotVel2D( R : integer) : TState2D');
 CL.AddDelphiFunction('Function GetRobotCenterOfMass( R : integer) : TPoint3D');
 CL.AddDelphiFunction('Function GetSolidIndex( R : integer; ID : string) : integer');
 CL.AddDelphiFunction('Procedure SetSolidMass( R, i : integer; nmass : double)');
 CL.AddDelphiFunction('Function GetSolidMass( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetSolidCenterOfMass( R, i : integer) : TPoint3D');
 CL.AddDelphiFunction('Procedure SetSolidPos( R, i : integer; x, y, z : double)');
 CL.AddDelphiFunction('Procedure SetSolidPosMat( R, i : integer; P : Matrix)');
 CL.AddDelphiFunction('Procedure SetSolidRotationMat( R, i : integer; Rot : Matrix)');
 CL.AddDelphiFunction('Function GetSolidPos( R, i : integer) : TPoint3D');
 CL.AddDelphiFunction('Function GetSolidLinearVel( R, i : integer) : TPoint3D');
 CL.AddDelphiFunction('Function GetSolidPosMat( R, i : integer) : Matrix');
 CL.AddDelphiFunction('Function GetSolidLinearVelMat( R, i : integer) : Matrix');
 CL.AddDelphiFunction('Function GetSolidRotMat( R, i : integer) : Matrix');
 CL.AddDelphiFunction('Function GetSolidAgularVelMat( R, i : integer) : Matrix');
 CL.AddDelphiFunction('Function GetRobotX( R : integer) : double');
 CL.AddDelphiFunction('Function GetRobotY( R : integer) : double');
 CL.AddDelphiFunction('Function GetRobotTheta( R : integer) : double');
 CL.AddDelphiFunction('Function GetRobotVx( R : integer) : double');
 CL.AddDelphiFunction('Function GetRobotVy( R : integer) : double');
 CL.AddDelphiFunction('Function GetRobotW( R : integer) : double');
 CL.AddDelphiFunction('Function GetSolidX( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetSolidY( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetSolidZ( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetSolidTheta( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetSolidVx( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetSolidVy( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetSolidVz( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetSolidColor( R, i : integer) : TRGBAColor');
 CL.AddDelphiFunction('Procedure SetSolidColor( R, I : integer; Red, Green, Blue : byte)');
 CL.AddDelphiFunction('Function GetSolidCanvas( R, i : integer) : TCanvas');
 CL.AddDelphiFunction('Procedure SolidCanvasClear( R, i : integer)');
 CL.AddDelphiFunction('Function GetPaintTargetExtents( i : integer) : TExtents');
 CL.AddDelphiFunction('Procedure SetSolidSurfaceFriction( R, i : integer; mu, mu2 : double)');
 CL.AddDelphiFunction('Procedure SetSolidForce( R, i : integer; Fx, Fy, Fz : double)');
 CL.AddDelphiFunction('Function GetSolidSize( R, i : integer) : TPoint3D');
 CL.AddDelphiFunction('Procedure SetSolidSize( R, i : integer; x, y, z : double)');
 CL.AddDelphiFunction('Function GetGlobalSensorIndex( ID : string) : integer');
 CL.AddDelphiFunction('Function GetGlobalSensorVal( i : integer) : double');
 CL.AddDelphiFunction('Function GetGlobalSensorValues( i : integer) : Matrix');
 CL.AddDelphiFunction('Procedure SetGlobalSensorVin( i : integer; Vin : byte)');
 CL.AddDelphiFunction('Procedure SetSensorVin( R, i : integer; Vin : byte)');
 CL.AddDelphiFunction('Function GetSensorIndex( R : integer; ID : string) : integer');
 CL.AddDelphiFunction('Function GetSensorVal( R, i : integer) : double');
 CL.AddDelphiFunction('Procedure SetSensorColor( R, i : integer; Red, Green, Blue : byte)');
 CL.AddDelphiFunction('Function GetSensorValues( R, i : integer) : Matrix');
 CL.AddDelphiFunction('Function GetThingIndex( ID : string) : integer');
 CL.AddDelphiFunction('Function GetThingsCount : integer');
 CL.AddDelphiFunction('Function GetThingColor( T, c : integer) : TRGBAColor');
 CL.AddDelphiFunction('Procedure SetThingColor( T, c : integer; Red, Green, Blue : byte)');
 CL.AddDelphiFunction('Function GetThingPos( T : integer) : TPoint3D');
 CL.AddDelphiFunction('Procedure SetThingPos( T : integer; x, y, z : double)');
 CL.AddDelphiFunction('Function GetThingRotMat( T : integer) : Matrix');
 CL.AddDelphiFunction('Procedure SetThingRotationMat( T : integer; Rot : Matrix)');
 CL.AddDelphiFunction('Function GetThingAgularVelMat( T : integer) : Matrix');
 CL.AddDelphiFunction('Function GetThingSize( T : integer) : TPoint3D');
 CL.AddDelphiFunction('Procedure SetThingSize( T : integer; x, y, z : double)');
 CL.AddDelphiFunction('Function AddThingBox( ID : string; mass, posx, posY, posZ, sizeX, sizeY, sizeZ : double; rgb24 : integer) : integer');
 CL.AddDelphiFunction('Function AddThingSphere( ID : string; mass, posX, posY, posZ, radius : double; rgb24 : integer) : integer');
 CL.AddDelphiFunction('Function AddThingCylinder( ID : string; mass, posx, posY, posZ, radius, len : double; rgb24 : integer) : integer');
 CL.AddDelphiFunction('Function DeleteThing( ID : string) : integer');
 CL.AddDelphiFunction('Procedure MeshThing( ID, MeshFile, MeshShadowFile : string; MeshScale : double; MeshCastsShadows : boolean)');
 CL.AddDelphiFunction('Procedure SetThingForce( T : integer; Fx, Fy, Fz : double)');
 CL.AddDelphiFunction('Procedure SetThingForceAtRelPos( T : integer; Fx, Fy, Fz, Px, Py, Pz : double)');
 CL.AddDelphiFunction('Procedure SetThingRelForce( T : integer; Fx, Fy, Fz : double)');
 CL.AddDelphiFunction('Procedure SetThingRelForceAtRelPos( T : integer; Fx, Fy, Fz, Px, Py, Pz : double)');
 CL.AddDelphiFunction('Procedure SetThingSurfaceMu( T : integer; mu, mu2 : double)');
 CL.AddDelphiFunction('Function GetThingSpeed( T : integer) : TPoint3D');
 CL.AddDelphiFunction('Procedure SetThingSpeed( T : integer; vx, vy, vz : double)');
 CL.AddDelphiFunction('Function GetThingAngularVel( T : integer) : TPoint3D');
 CL.AddDelphiFunction('Procedure ClearThings');
 CL.AddDelphiFunction('Function GetShellPos( R, i : integer) : TPoint3D');
 CL.AddDelphiFunction('Procedure SetShellColor( R, i : integer; Red, Green, Blue : byte)');
 CL.AddDelphiFunction('Function GetShellColor( R, i : integer) : TRGBAColor');
 CL.AddDelphiFunction('Function GetObstacleIndex( ID : string) : integer');
 CL.AddDelphiFunction('Procedure SetObstacleColor( I : integer; Red, Green, Blue : byte)');
 CL.AddDelphiFunction('Function GetObstacleColor( I : integer) : TRGBAColor');
 CL.AddDelphiFunction('Function GetObstaclePos( T : integer) : TPoint3D');
 CL.AddDelphiFunction('Procedure SetObstaclePos( T : integer; x, y, z : double)');
 CL.AddDelphiFunction('Function GetObstacleRotMat( T : integer) : Matrix');
 CL.AddDelphiFunction('Procedure SetObstacleRotationMat( T : integer; Rot : Matrix)');
 CL.AddDelphiFunction('Function AddOBstacleBox( ID : string; posx, posY, posZ, sizeX, sizeY, sizeZ : double; rgb24 : integer) : integer');
 CL.AddDelphiFunction('Procedure ClearObstacles');
 CL.AddDelphiFunction('Function GetAxisOdo( R, i : integer) : integer');
 CL.AddDelphiFunction('Function GetAxisState( R, i : integer) : TAxisState');
 CL.AddDelphiFunction('Function GetAxisPos( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetAxisSpeed( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetAxisTorque( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetAxisI( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetAxisU( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetAxisUIPower( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetAxisTWPower( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetAxisStateRef( R, i : integer) : TAxisState');
 CL.AddDelphiFunction('Function GetAxisPosRef( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetAxisSpeedRef( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetAxisMotorSpeed( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetAxisMotorPos( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetAxisMotorPosDeg( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetAxisEnergy( R, i : integer) : double');
 CL.AddDelphiFunction('Procedure ResetAxisEnergy( R, i : integer)');
 CL.AddDelphiFunction('Procedure SetMotorPars( R, i : integer; aMotorPars : TMotorPars)');
 CL.AddDelphiFunction('Function GetMotorPars( R, i : integer) : TMotorPars');
 CL.AddDelphiFunction('Procedure SetMotorControllerPars( R, i : integer; nKi, nKd, nKp, nKf : double)');
 CL.AddDelphiFunction('Function GetMotorControllerPars( R, i : integer) : TMotorControllerPars');
 CL.AddDelphiFunction('Procedure SetMotorControllerMode( R, i : integer; newMode : string)');
 CL.AddDelphiFunction('Function GetMotorControllerMode( R, i : integer) : string');
 CL.AddDelphiFunction('Procedure SetMotorControllerState( R, i : integer; newState : boolean)');
 CL.AddDelphiFunction('Function GetMotorControllerState( R, i : integer) : boolean');
 CL.AddDelphiFunction('Procedure SetMotorActive( R, i : integer; nState : boolean)');
 CL.AddDelphiFunction('Function IsMotorActive( R, i : integer) : boolean');
 CL.AddDelphiFunction('Procedure SetFrictionDef( R, i : integer; nBv, nFc, nCoulombLimit : double)');
 CL.AddDelphiFunction('Function GetFrictionDef( R, i : integer) : TFrictionDef');
 CL.AddDelphiFunction('Procedure SetBeltSpeed( R, i : integer; nSpeed : double)');
 CL.AddDelphiFunction('Function GetBeltSpeed( R, i : integer) : double');
 CL.AddDelphiFunction('Procedure SetAxisSpring( R, i : integer; k, ZeroPos : double)');
 CL.AddDelphiFunction('Procedure SetAxisStateRef( R, i : integer; aState : TAxisState)');
 CL.AddDelphiFunction('Procedure SetAxisPosRef( R, i : integer; aPos : double)');
 CL.AddDelphiFunction('Procedure SetAxisSpeedRef( R, i : integer; aSpeed : double)');
 CL.AddDelphiFunction('Procedure SetAxisVoltageRef( R, i : integer; aVoltage : double)');
 CL.AddDelphiFunction('Procedure SetAxisTorqueRef( R, i : integer; aTorque : double)');
 CL.AddDelphiFunction('Function GetAxisPosDeg( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetAxisSpeedDeg( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetAxisPosRefDeg( R, i : integer) : double');
 CL.AddDelphiFunction('Function GetAxisSpeedRefDeg( R, i : integer) : double');
 CL.AddDelphiFunction('Function Deg( angle : double) : double');
 CL.AddDelphiFunction('Function Rad( angle : double) : double');
 CL.AddDelphiFunction('Function GetAxisIndex( R : integer; ID : string; i : integer) : integer');
 CL.AddDelphiFunction('Procedure LoadJointWayPoints( r : integer; JointPointsFileName : string)');
 CL.AddDelphiFunction('Procedure SaveJointWayPoints( r : integer; JointPointsFileName : string)');
 CL.AddDelphiFunction('Function CountJointWayPoints( R, i : integer) : integer');
 CL.AddDelphiFunction('Function GetJointWayPoint( R, i, idx : integer) : TAxisPoint');
 CL.AddDelphiFunction('Procedure SetJointWayPoint( R, i, idx : integer; apos, aspeed, atime : double)');
 CL.AddDelphiFunction('Function GetAxisTrajPoint( R, i, idx : integer) : TAxisPoint');
 CL.AddDelphiFunction('Procedure SetAxisTrajPoint( R, i, idx : integer; LP : TAxisPoint)');
 CL.AddDelphiFunction('Procedure AddAxisTrajPoint( R, i : integer; LP : TAxisPoint)');
 CL.AddDelphiFunction('Procedure DelAxisTrajPoint( R, i, idx : integer)');
 CL.AddDelphiFunction('Function CountAxisTrajPoints( R, i : integer) : integer');
 CL.AddDelphiFunction('Procedure ClearAxisTrajPoints( R, i : integer)');
 CL.AddDelphiFunction('Procedure SetTrailColor( T : integer; Red, Green, Blue : byte)');
 CL.AddDelphiFunction('Procedure AddTrailNode( T : integer; x, y, z : double)');
 CL.AddDelphiFunction('Procedure DelTrailNode( T : integer)');
 CL.AddDelphiFunction('Procedure ClearTrail( T : integer)');
 CL.AddDelphiFunction('Function KeyPressed( k : integer) : Boolean');
 CL.AddConstantN('VK_LBUTTON','LongInt').SetInt( 1);
 CL.AddConstantN('VK_RBUTTON','LongInt').SetInt( 2);
 CL.AddConstantN('VK_CANCEL','LongInt').SetInt( 3);
 CL.AddConstantN('VK_MBUTTON','LongInt').SetInt( 4);
 CL.AddConstantN('VK_BACK','LongInt').SetInt( 8);
 CL.AddConstantN('VK_TAB','LongInt').SetInt( 9);
 CL.AddConstantN('VK_CLEAR','LongInt').SetInt( 12);
 CL.AddConstantN('VK_RETURN','LongInt').SetInt( 13);
 CL.AddConstantN('VK_SHIFT','LongWord').SetUInt( $10);
 CL.AddConstantN('VK_CONTROL','LongInt').SetInt( 17);
 CL.AddConstantN('VK_MENU','LongInt').SetInt( 18);
 CL.AddConstantN('VK_PAUSE','LongInt').SetInt( 19);
 CL.AddConstantN('VK_CAPITAL','LongInt').SetInt( 20);
 CL.AddConstantN('VK_KANA','LongInt').SetInt( 21);
 CL.AddConstantN('VK_HANGUL','LongInt').SetInt( 21);
 CL.AddConstantN('VK_JUNJA','LongInt').SetInt( 23);
 CL.AddConstantN('VK_FINAL','LongInt').SetInt( 24);
 CL.AddConstantN('VK_HANJA','LongInt').SetInt( 25);
 CL.AddConstantN('VK_KANJI','LongInt').SetInt( 25);
 CL.AddConstantN('VK_CONVERT','LongInt').SetInt( 28);
 CL.AddConstantN('VK_NONCONVERT','LongInt').SetInt( 29);
 CL.AddConstantN('VK_ACCEPT','LongInt').SetInt( 30);
 CL.AddConstantN('VK_MODECHANGE','LongInt').SetInt( 31);
 CL.AddConstantN('VK_ESCAPE','LongInt').SetInt( 27);
 CL.AddConstantN('VK_SPACE','LongWord').SetUInt( $20);
 CL.AddConstantN('VK_PRIOR','LongInt').SetInt( 33);
 CL.AddConstantN('VK_NEXT','LongInt').SetInt( 34);
 CL.AddConstantN('VK_END','LongInt').SetInt( 35);
 CL.AddConstantN('VK_HOME','LongInt').SetInt( 36);
 CL.AddConstantN('VK_LEFT','LongInt').SetInt( 37);
 CL.AddConstantN('VK_UP','LongInt').SetInt( 38);
 CL.AddConstantN('VK_RIGHT','LongInt').SetInt( 39);
 CL.AddConstantN('VK_DOWN','LongInt').SetInt( 40);
 CL.AddConstantN('VK_SELECT','LongInt').SetInt( 41);
 CL.AddConstantN('VK_PRINT','LongInt').SetInt( 42);
 CL.AddConstantN('VK_EXECUTE','LongInt').SetInt( 43);
 CL.AddConstantN('VK_SNAPSHOT','LongInt').SetInt( 44);
 CL.AddConstantN('VK_INSERT','LongInt').SetInt( 45);
 CL.AddConstantN('VK_DELETE','LongInt').SetInt( 46);
 CL.AddConstantN('VK_HELP','LongInt').SetInt( 47);
 CL.AddConstantN('VK_LWIN','LongInt').SetInt( 91);
 CL.AddConstantN('VK_RWIN','LongInt').SetInt( 92);
 CL.AddConstantN('VK_APPS','LongInt').SetInt( 93);
 CL.AddConstantN('VK_NUMPAD0','LongInt').SetInt( 96);
 CL.AddConstantN('VK_NUMPAD1','LongInt').SetInt( 97);
 CL.AddConstantN('VK_NUMPAD2','LongInt').SetInt( 98);
 CL.AddConstantN('VK_NUMPAD3','LongInt').SetInt( 99);
 CL.AddConstantN('VK_NUMPAD4','LongInt').SetInt( 100);
 CL.AddConstantN('VK_NUMPAD5','LongInt').SetInt( 101);
 CL.AddConstantN('VK_NUMPAD6','LongInt').SetInt( 102);
 CL.AddConstantN('VK_NUMPAD7','LongInt').SetInt( 103);
 CL.AddConstantN('VK_NUMPAD8','LongInt').SetInt( 104);
 CL.AddConstantN('VK_NUMPAD9','LongInt').SetInt( 105);
 CL.AddConstantN('VK_MULTIPLY','LongInt').SetInt( 106);
 CL.AddConstantN('VK_ADD','LongInt').SetInt( 107);
 CL.AddConstantN('VK_SEPARATOR','LongInt').SetInt( 108);
 CL.AddConstantN('VK_SUBTRACT','LongInt').SetInt( 109);
 CL.AddConstantN('VK_DECIMAL','LongInt').SetInt( 110);
 CL.AddConstantN('VK_DIVIDE','LongInt').SetInt( 111);
 CL.AddConstantN('VK_F1','LongInt').SetInt( 112);
 CL.AddConstantN('VK_F2','LongInt').SetInt( 113);
 CL.AddConstantN('VK_F3','LongInt').SetInt( 114);
 CL.AddConstantN('VK_F4','LongInt').SetInt( 115);
 CL.AddConstantN('VK_F5','LongInt').SetInt( 116);
 CL.AddConstantN('VK_F6','LongInt').SetInt( 117);
 CL.AddConstantN('VK_F7','LongInt').SetInt( 118);
 CL.AddConstantN('VK_F8','LongInt').SetInt( 119);
 CL.AddConstantN('VK_F9','LongInt').SetInt( 120);
 CL.AddConstantN('VK_F10','LongInt').SetInt( 121);
 CL.AddConstantN('VK_F11','LongInt').SetInt( 122);
 CL.AddConstantN('VK_F12','LongInt').SetInt( 123);
 CL.AddConstantN('VK_F13','LongInt').SetInt( 124);
 CL.AddConstantN('VK_F14','LongInt').SetInt( 125);
 CL.AddConstantN('VK_F15','LongInt').SetInt( 126);
 CL.AddConstantN('VK_F16','LongInt').SetInt( 127);
 CL.AddConstantN('VK_F17','LongInt').SetInt( 128);
 CL.AddConstantN('VK_F18','LongInt').SetInt( 129);
 CL.AddConstantN('VK_F19','LongInt').SetInt( 130);
 CL.AddConstantN('VK_F20','LongInt').SetInt( 131);
 CL.AddConstantN('VK_F21','LongInt').SetInt( 132);
 CL.AddConstantN('VK_F22','LongInt').SetInt( 133);
 CL.AddConstantN('VK_F23','LongInt').SetInt( 134);
 CL.AddConstantN('VK_F24','LongInt').SetInt( 135);
 CL.AddConstantN('VK_NUMLOCK','LongInt').SetInt( 144);
 CL.AddConstantN('VK_SCROLL','LongInt').SetInt( 145);
 CL.AddConstantN('VK_LSHIFT','LongInt').SetInt( 160);
 CL.AddConstantN('VK_RSHIFT','LongInt').SetInt( 161);
 CL.AddConstantN('VK_LCONTROL','LongInt').SetInt( 162);
 CL.AddConstantN('VK_RCONTROL','LongInt').SetInt( 163);
 CL.AddConstantN('VK_LMENU','LongInt').SetInt( 164);
 CL.AddConstantN('VK_RMENU','LongInt').SetInt( 165);
 CL.AddConstantN('VK_PROCESSKEY','LongInt').SetInt( 229);
 CL.AddConstantN('VK_ATTN','LongInt').SetInt( 246);
 CL.AddConstantN('VK_CRSEL','LongInt').SetInt( 247);
 CL.AddConstantN('VK_EXSEL','LongInt').SetInt( 248);
 CL.AddConstantN('VK_EREOF','LongInt').SetInt( 249);
 CL.AddConstantN('VK_PLAY','LongInt').SetInt( 250);
 CL.AddConstantN('VK_ZOOM','LongInt').SetInt( 251);
 CL.AddConstantN('VK_NONAME','LongInt').SetInt( 252);
 CL.AddConstantN('VK_PA1','LongInt').SetInt( 253);
 CL.AddConstantN('VK_OEM_CLEAR','LongInt').SetInt( 254);
end;

(* === run-time registration functions === *)
(*----------------------------------------------------------------------------*)
procedure RIRegister_ODERobotsPublished_Routines(S: TPSExec);
begin
 S.RegisterDelphiFunction(@SetFireScale, 'SetFireScale', cdRegister);
 S.RegisterDelphiFunction(@SetFirePosition, 'SetFirePosition', cdRegister);
 S.RegisterDelphiFunction(@StartFire, 'StartFire', cdRegister);
 S.RegisterDelphiFunction(@StopFire, 'StopFire', cdRegister);
 S.RegisterDelphiFunction(@StopSolidFire, 'StopSolidFire', cdRegister);
 S.RegisterDelphiFunction(@StartSolidFire, 'StartSolidFire', cdRegister);
 S.RegisterDelphiFunction(@GetSceneConstant, 'GetSceneConstant', cdRegister);
 S.RegisterDelphiFunction(@SetRobotPos, 'SetRobotPos', cdRegister);
 S.RegisterDelphiFunction(@GetRobotIndex, 'GetRobotIndex', cdRegister);
 S.RegisterDelphiFunction(@GetRobotPos2D, 'GetRobotPos2D', cdRegister);
 S.RegisterDelphiFunction(@GetRobotVel2D, 'GetRobotVel2D', cdRegister);
 S.RegisterDelphiFunction(@GetRobotCenterOfMass, 'GetRobotCenterOfMass', cdRegister);
 S.RegisterDelphiFunction(@GetSolidIndex, 'GetSolidIndex', cdRegister);
 S.RegisterDelphiFunction(@SetSolidMass, 'SetSolidMass', cdRegister);
 S.RegisterDelphiFunction(@GetSolidMass, 'GetSolidMass', cdRegister);
 S.RegisterDelphiFunction(@GetSolidCenterOfMass, 'GetSolidCenterOfMass', cdRegister);
 S.RegisterDelphiFunction(@SetSolidPos, 'SetSolidPos', cdRegister);
 S.RegisterDelphiFunction(@SetSolidPosMat, 'SetSolidPosMat', cdRegister);
 S.RegisterDelphiFunction(@SetSolidRotationMat, 'SetSolidRotationMat', cdRegister);
 S.RegisterDelphiFunction(@GetSolidPos, 'GetSolidPos', cdRegister);
 S.RegisterDelphiFunction(@GetSolidLinearVel, 'GetSolidLinearVel', cdRegister);
 S.RegisterDelphiFunction(@GetSolidPosMat, 'GetSolidPosMat', cdRegister);
 S.RegisterDelphiFunction(@GetSolidLinearVelMat, 'GetSolidLinearVelMat', cdRegister);
 S.RegisterDelphiFunction(@GetSolidRotMat, 'GetSolidRotMat', cdRegister);
 S.RegisterDelphiFunction(@GetSolidAgularVelMat, 'GetSolidAgularVelMat', cdRegister);
 S.RegisterDelphiFunction(@GetRobotX, 'GetRobotX', cdRegister);
 S.RegisterDelphiFunction(@GetRobotY, 'GetRobotY', cdRegister);
 S.RegisterDelphiFunction(@GetRobotTheta, 'GetRobotTheta', cdRegister);
 S.RegisterDelphiFunction(@GetRobotVx, 'GetRobotVx', cdRegister);
 S.RegisterDelphiFunction(@GetRobotVy, 'GetRobotVy', cdRegister);
 S.RegisterDelphiFunction(@GetRobotW, 'GetRobotW', cdRegister);
 S.RegisterDelphiFunction(@GetSolidX, 'GetSolidX', cdRegister);
 S.RegisterDelphiFunction(@GetSolidY, 'GetSolidY', cdRegister);
 S.RegisterDelphiFunction(@GetSolidZ, 'GetSolidZ', cdRegister);
 S.RegisterDelphiFunction(@GetSolidTheta, 'GetSolidTheta', cdRegister);
 S.RegisterDelphiFunction(@GetSolidVx, 'GetSolidVx', cdRegister);
 S.RegisterDelphiFunction(@GetSolidVy, 'GetSolidVy', cdRegister);
 S.RegisterDelphiFunction(@GetSolidVz, 'GetSolidVz', cdRegister);
 S.RegisterDelphiFunction(@GetSolidColor, 'GetSolidColor', cdRegister);
 S.RegisterDelphiFunction(@SetSolidColor, 'SetSolidColor', cdRegister);
 S.RegisterDelphiFunction(@GetSolidCanvas, 'GetSolidCanvas', cdRegister);
 S.RegisterDelphiFunction(@SolidCanvasClear, 'SolidCanvasClear', cdRegister);
 S.RegisterDelphiFunction(@GetPaintTargetExtents, 'GetPaintTargetExtents', cdRegister);
 S.RegisterDelphiFunction(@SetSolidSurfaceFriction, 'SetSolidSurfaceFriction', cdRegister);
 S.RegisterDelphiFunction(@SetSolidForce, 'SetSolidForce', cdRegister);
 S.RegisterDelphiFunction(@GetSolidSize, 'GetSolidSize', cdRegister);
 S.RegisterDelphiFunction(@SetSolidSize, 'SetSolidSize', cdRegister);
 S.RegisterDelphiFunction(@GetGlobalSensorIndex, 'GetGlobalSensorIndex', cdRegister);
 S.RegisterDelphiFunction(@GetGlobalSensorVal, 'GetGlobalSensorVal', cdRegister);
 S.RegisterDelphiFunction(@GetGlobalSensorValues, 'GetGlobalSensorValues', cdRegister);
 S.RegisterDelphiFunction(@SetGlobalSensorVin, 'SetGlobalSensorVin', cdRegister);
 S.RegisterDelphiFunction(@SetSensorVin, 'SetSensorVin', cdRegister);
 S.RegisterDelphiFunction(@GetSensorIndex, 'GetSensorIndex', cdRegister);
 S.RegisterDelphiFunction(@GetSensorVal, 'GetSensorVal', cdRegister);
 S.RegisterDelphiFunction(@SetSensorColor, 'SetSensorColor', cdRegister);
 S.RegisterDelphiFunction(@GetSensorValues, 'GetSensorValues', cdRegister);
 S.RegisterDelphiFunction(@GetThingIndex, 'GetThingIndex', cdRegister);
 S.RegisterDelphiFunction(@GetThingsCount, 'GetThingsCount', cdRegister);
 S.RegisterDelphiFunction(@GetThingColor, 'GetThingColor', cdRegister);
 S.RegisterDelphiFunction(@SetThingColor, 'SetThingColor', cdRegister);
 S.RegisterDelphiFunction(@GetThingPos, 'GetThingPos', cdRegister);
 S.RegisterDelphiFunction(@SetThingPos, 'SetThingPos', cdRegister);
 S.RegisterDelphiFunction(@GetThingRotMat, 'GetThingRotMat', cdRegister);
 S.RegisterDelphiFunction(@SetThingRotationMat, 'SetThingRotationMat', cdRegister);
 S.RegisterDelphiFunction(@GetThingAgularVelMat, 'GetThingAgularVelMat', cdRegister);
 S.RegisterDelphiFunction(@GetThingSize, 'GetThingSize', cdRegister);
 S.RegisterDelphiFunction(@SetThingSize, 'SetThingSize', cdRegister);
 S.RegisterDelphiFunction(@AddThingBox, 'AddThingBox', cdRegister);
 S.RegisterDelphiFunction(@AddThingSphere, 'AddThingSphere', cdRegister);
 S.RegisterDelphiFunction(@AddThingCylinder, 'AddThingCylinder', cdRegister);
 S.RegisterDelphiFunction(@DeleteThing, 'DeleteThing', cdRegister);
 S.RegisterDelphiFunction(@MeshThing, 'MeshThing', cdRegister);
 S.RegisterDelphiFunction(@SetThingForce, 'SetThingForce', cdRegister);
 S.RegisterDelphiFunction(@SetThingForceAtRelPos, 'SetThingForceAtRelPos', cdRegister);
 S.RegisterDelphiFunction(@SetThingRelForce, 'SetThingRelForce', cdRegister);
 S.RegisterDelphiFunction(@SetThingRelForceAtRelPos, 'SetThingRelForceAtRelPos', cdRegister);
 S.RegisterDelphiFunction(@SetThingSurfaceMu, 'SetThingSurfaceMu', cdRegister);
 S.RegisterDelphiFunction(@GetThingSpeed, 'GetThingSpeed', cdRegister);
 S.RegisterDelphiFunction(@SetThingSpeed, 'SetThingSpeed', cdRegister);
 S.RegisterDelphiFunction(@GetThingAngularVel, 'GetThingAngularVel', cdRegister);
 S.RegisterDelphiFunction(@ClearThings, 'ClearThings', cdRegister);
 S.RegisterDelphiFunction(@GetShellPos, 'GetShellPos', cdRegister);
 S.RegisterDelphiFunction(@SetShellColor, 'SetShellColor', cdRegister);
 S.RegisterDelphiFunction(@GetShellColor, 'GetShellColor', cdRegister);
 S.RegisterDelphiFunction(@GetObstacleIndex, 'GetObstacleIndex', cdRegister);
 S.RegisterDelphiFunction(@SetObstacleColor, 'SetObstacleColor', cdRegister);
 S.RegisterDelphiFunction(@GetObstacleColor, 'GetObstacleColor', cdRegister);
 S.RegisterDelphiFunction(@GetObstaclePos, 'GetObstaclePos', cdRegister);
 S.RegisterDelphiFunction(@SetObstaclePos, 'SetObstaclePos', cdRegister);
 S.RegisterDelphiFunction(@GetObstacleRotMat, 'GetObstacleRotMat', cdRegister);
 S.RegisterDelphiFunction(@SetObstacleRotationMat, 'SetObstacleRotationMat', cdRegister);
 S.RegisterDelphiFunction(@AddOBstacleBox, 'AddOBstacleBox', cdRegister);
 S.RegisterDelphiFunction(@ClearObstacles, 'ClearObstacles', cdRegister);
 S.RegisterDelphiFunction(@GetAxisOdo, 'GetAxisOdo', cdRegister);
 S.RegisterDelphiFunction(@GetAxisState, 'GetAxisState', cdRegister);
 S.RegisterDelphiFunction(@GetAxisPos, 'GetAxisPos', cdRegister);
 S.RegisterDelphiFunction(@GetAxisSpeed, 'GetAxisSpeed', cdRegister);
 S.RegisterDelphiFunction(@GetAxisTorque, 'GetAxisTorque', cdRegister);
 S.RegisterDelphiFunction(@GetAxisI, 'GetAxisI', cdRegister);
 S.RegisterDelphiFunction(@GetAxisU, 'GetAxisU', cdRegister);
 S.RegisterDelphiFunction(@GetAxisUIPower, 'GetAxisUIPower', cdRegister);
 S.RegisterDelphiFunction(@GetAxisTWPower, 'GetAxisTWPower', cdRegister);
 S.RegisterDelphiFunction(@GetAxisStateRef, 'GetAxisStateRef', cdRegister);
 S.RegisterDelphiFunction(@GetAxisPosRef, 'GetAxisPosRef', cdRegister);
 S.RegisterDelphiFunction(@GetAxisSpeedRef, 'GetAxisSpeedRef', cdRegister);
 S.RegisterDelphiFunction(@GetAxisMotorSpeed, 'GetAxisMotorSpeed', cdRegister);
 S.RegisterDelphiFunction(@GetAxisMotorPos, 'GetAxisMotorPos', cdRegister);
 S.RegisterDelphiFunction(@GetAxisMotorPosDeg, 'GetAxisMotorPosDeg', cdRegister);
 S.RegisterDelphiFunction(@GetAxisEnergy, 'GetAxisEnergy', cdRegister);
 S.RegisterDelphiFunction(@ResetAxisEnergy, 'ResetAxisEnergy', cdRegister);
 S.RegisterDelphiFunction(@SetMotorPars, 'SetMotorPars', cdRegister);
 S.RegisterDelphiFunction(@GetMotorPars, 'GetMotorPars', cdRegister);
 S.RegisterDelphiFunction(@SetMotorControllerPars, 'SetMotorControllerPars', cdRegister);
 S.RegisterDelphiFunction(@GetMotorControllerPars, 'GetMotorControllerPars', cdRegister);
 S.RegisterDelphiFunction(@SetMotorControllerMode, 'SetMotorControllerMode', cdRegister);
 S.RegisterDelphiFunction(@GetMotorControllerMode, 'GetMotorControllerMode', cdRegister);
 S.RegisterDelphiFunction(@SetMotorControllerState, 'SetMotorControllerState', cdRegister);
 S.RegisterDelphiFunction(@GetMotorControllerState, 'GetMotorControllerState', cdRegister);
 S.RegisterDelphiFunction(@SetMotorActive, 'SetMotorActive', cdRegister);
 S.RegisterDelphiFunction(@IsMotorActive, 'IsMotorActive', cdRegister);
 S.RegisterDelphiFunction(@SetFrictionDef, 'SetFrictionDef', cdRegister);
 S.RegisterDelphiFunction(@GetFrictionDef, 'GetFrictionDef', cdRegister);
 S.RegisterDelphiFunction(@SetBeltSpeed, 'SetBeltSpeed', cdRegister);
 S.RegisterDelphiFunction(@GetBeltSpeed, 'GetBeltSpeed', cdRegister);
 S.RegisterDelphiFunction(@SetAxisSpring, 'SetAxisSpring', cdRegister);
 S.RegisterDelphiFunction(@SetAxisStateRef, 'SetAxisStateRef', cdRegister);
 S.RegisterDelphiFunction(@SetAxisPosRef, 'SetAxisPosRef', cdRegister);
 S.RegisterDelphiFunction(@SetAxisSpeedRef, 'SetAxisSpeedRef', cdRegister);
 S.RegisterDelphiFunction(@SetAxisVoltageRef, 'SetAxisVoltageRef', cdRegister);
 S.RegisterDelphiFunction(@SetAxisTorqueRef, 'SetAxisTorqueRef', cdRegister);
 S.RegisterDelphiFunction(@GetAxisPosDeg, 'GetAxisPosDeg', cdRegister);
 S.RegisterDelphiFunction(@GetAxisSpeedDeg, 'GetAxisSpeedDeg', cdRegister);
 S.RegisterDelphiFunction(@GetAxisPosRefDeg, 'GetAxisPosRefDeg', cdRegister);
 S.RegisterDelphiFunction(@GetAxisSpeedRefDeg, 'GetAxisSpeedRefDeg', cdRegister);
 S.RegisterDelphiFunction(@Deg, 'Deg', cdRegister);
 S.RegisterDelphiFunction(@Rad, 'Rad', cdRegister);
 S.RegisterDelphiFunction(@GetAxisIndex, 'GetAxisIndex', cdRegister);
 S.RegisterDelphiFunction(@LoadJointWayPoints, 'LoadJointWayPoints', cdRegister);
 S.RegisterDelphiFunction(@SaveJointWayPoints, 'SaveJointWayPoints', cdRegister);
 S.RegisterDelphiFunction(@CountJointWayPoints, 'CountJointWayPoints', cdRegister);
 S.RegisterDelphiFunction(@GetJointWayPoint, 'GetJointWayPoint', cdRegister);
 S.RegisterDelphiFunction(@SetJointWayPoint, 'SetJointWayPoint', cdRegister);
 S.RegisterDelphiFunction(@GetAxisTrajPoint, 'GetAxisTrajPoint', cdRegister);
 S.RegisterDelphiFunction(@SetAxisTrajPoint, 'SetAxisTrajPoint', cdRegister);
 S.RegisterDelphiFunction(@AddAxisTrajPoint, 'AddAxisTrajPoint', cdRegister);
 S.RegisterDelphiFunction(@DelAxisTrajPoint, 'DelAxisTrajPoint', cdRegister);
 S.RegisterDelphiFunction(@CountAxisTrajPoints, 'CountAxisTrajPoints', cdRegister);
 S.RegisterDelphiFunction(@ClearAxisTrajPoints, 'ClearAxisTrajPoints', cdRegister);
 S.RegisterDelphiFunction(@SetTrailColor, 'SetTrailColor', cdRegister);
 S.RegisterDelphiFunction(@AddTrailNode, 'AddTrailNode', cdRegister);
 S.RegisterDelphiFunction(@DelTrailNode, 'DelTrailNode', cdRegister);
 S.RegisterDelphiFunction(@ClearTrail, 'ClearTrail', cdRegister);
 S.RegisterDelphiFunction(@KeyPressed, 'KeyPressed', cdRegister);
end;



{ TPSImport_ODERobotsPublished }
(*----------------------------------------------------------------------------*)
procedure TPSImport_ODERobotsPublished.CompileImport1(CompExec: TPSScript);
begin
  SIRegister_ODERobotsPublished(CompExec.Comp);
end;
(*----------------------------------------------------------------------------*)
procedure TPSImport_ODERobotsPublished.ExecImport1(CompExec: TPSScript; const ri: TPSRuntimeClassImporter);
begin
  //RIRegister_ODERobotsPublished(ri);
  RIRegister_ODERobotsPublished_Routines(CompExec.Exec); // comment it if no routines
end;
(*----------------------------------------------------------------------------*)


end.
