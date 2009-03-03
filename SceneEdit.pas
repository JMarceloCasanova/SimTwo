unit SceneEdit;

interface

uses
  Windows, Messages, SysUtils, Variants, Classes, Graphics, Controls, Forms,
  Dialogs, ComCtrls, SynEditHighlighter, SynHighlighterXML, SynEdit, Menus,
  ExtCtrls, StdCtrls, ShellAPI, SynEditTypes, SynEditMiscClasses, SynEditSearch,
  rxPlacemnt, ProjConfig;

type
  TFXMLEdit = class(TForm)
    StatusBar: TStatusBar;
    LBErrors: TListBox;
    Splitter: TSplitter;
    MainMenu: TMainMenu;
    MenuFile: TMenuItem;
    MenuNew: TMenuItem;
    MenuOpen: TMenuItem;
    N2: TMenuItem;
    MenuSave: TMenuItem;
    MenuSaveAs: TMenuItem;
    N3: TMenuItem;
    MenuPrintSource: TMenuItem;
    N1: TMenuItem;
    MenuExit: TMenuItem;
    MenuEdit: TMenuItem;
    MenuUndo: TMenuItem;
    MenuRedo: TMenuItem;
    N4: TMenuItem;
    MenuFind: TMenuItem;
    MenuReplace: TMenuItem;
    MenuScene: TMenuItem;
    MenuReBuild: TMenuItem;
    MenuWindow: TMenuItem;
    MenuSceneXML: TMenuItem;
    SynXMLSyn: TSynXMLSyn;
    PageControlXML: TPageControl;
    TabScene: TTabSheet;
    SynEditXML: TSynEdit;
    ReplaceDialog: TReplaceDialog;
    FindDialog: TFindDialog;
    SynEditSearch: TSynEditSearch;
    OpenDialog: TOpenDialog;
    SaveDialog: TSaveDialog;
    FormStorage: TFormStorage;
    procedure FormShow(Sender: TObject);
    procedure MenuReBuildClick(Sender: TObject);
    procedure SynEditXMLStatusChange(Sender: TObject;
      Changes: TSynStatusChanges);
    procedure PageControlXMLChange(Sender: TObject);
    procedure MenuUndoClick(Sender: TObject);
    procedure MenuRedoClick(Sender: TObject);
    procedure MenuFindClick(Sender: TObject);
    procedure FindDialogFind(Sender: TObject);
    procedure MenuReplaceClick(Sender: TObject);
    procedure ReplaceDialogAction(Sender: TObject);
    procedure MenuSaveClick(Sender: TObject);
    procedure MenuSaveAsClick(Sender: TObject);
    procedure MenuNewClick(Sender: TObject);
    procedure MenuOpenClick(Sender: TObject);
    procedure MenuExitClick(Sender: TObject);
    procedure FormCreate(Sender: TObject);
  private
    function GetSynEdit: TSynEdit;
    function ActSynEdit: TSynEdit;
    procedure FindReplaceDialog(TmpSynEdit: TSynEdit; Dialog: TFindDialog);
    procedure CreateXMLTabEdit(SL: TStringList; i: integer);
    { Private declarations }
  public
    { Public declarations }
  end;

var
  FXMLEdit: TFXMLEdit;

implementation

uses Viewer;

{$R *.dfm}

procedure TFXMLEdit.CreateXMLTabEdit(SL: TStringList; i: integer);
var NewTabSheet: TTabSheet;
    NewSynEdit: TSynEdit;
begin
    NewTabSheet := TTabSheet.Create(Self);
    NewTabSheet.Name := 'TS' + inttostr(i);
    NewTabSheet.PageControl := PageControlXML;
    NewTabSheet.Caption := SL[i];
    NewSynEdit := TSynEdit.Create(Self);
    NewSynEdit.Name := 'SE' + inttostr(i);
    SL.Objects[i] := NewSynEdit;
    NewSynEdit.Parent := NewTabSheet;
    NewSynEdit.Align := alClient;
    NewSynEdit.Highlighter := SynXMLSyn;
    NewSynEdit.OnStatusChange := SynEditXMLStatusChange;
    NewSynEdit.Lines.LoadFromFile(SL[i]);
end;


procedure TFXMLEdit.FormShow(Sender: TObject);
var i: integer;
//    NewTabSheet: TTabSheet;
//    NewSynEdit: TSynEdit;
begin
  TabScene.TabVisible := false;
  SynEditXML.Lines.AddStrings(WorldODE.XMLFiles);
  for i := 0 to WorldODE.XMLFiles.Count -1 do begin
    CreateXMLTabEdit(WorldODE.XMLFiles, i);
  end;
end;

procedure TFXMLEdit.MenuReBuildClick(Sender: TObject);
var i: integer;
    TmpSynEdit: TSynEdit;
    prs: string;
begin
  for i := 0 to WorldODE.XMLFiles.Count -1 do begin
    if not (WorldODE.XMLFiles.Objects[i] is TSynEdit) then continue;
    TmpSynEdit := TSynEdit(WorldODE.XMLFiles.Objects[i]);
    if TmpSynEdit.Modified then begin
      TmpSynEdit.Lines.SaveToFile(WorldODE.XMLFiles[i]);
    end;
  end;

  prs := '';
  for i := 1 to ParamCount do begin
    prs := ' ' + ParamStr(i);
  end;
  FViewer.Close;
  ShellExecute(Handle, 'open', pchar(Application.ExeName), pchar(prs), nil,  SW_SHOWNORMAL);

end;

function TFXMLEdit.GetSynEdit: TSynEdit;
var i: integer;
begin
  i := PageControlXML.TabIndex;
  if (WorldODE.XMLFiles.Objects[i] is TSynEdit) then begin
    result := TSynEdit(WorldODE.XMLFiles.Objects[i]);
  end else result := nil;
end;

// Same as GetSynEdit but returns a dummy SynEdit instead of nil
function TFXMLEdit.ActSynEdit: TSynEdit;
var i: integer;
begin
  i := PageControlXML.TabIndex;
  if (WorldODE.XMLFiles.Objects[i] is TSynEdit) then begin
    result := TSynEdit(WorldODE.XMLFiles.Objects[i]);
  end else result := SynEditXML;
end;


procedure TFXMLEdit.SynEditXMLStatusChange(Sender: TObject; Changes: TSynStatusChanges);
var TmpSynEdit: TSynEdit;
begin
  TmpSynEdit := GetSynEdit();
  if TmpSynEdit = nil then exit;

  if (scCaretX in Changes) or (scCaretY in Changes) or (scAll in Changes) then
    StatusBar.Panels[0].Text := format('%6d: %3d',[TmpSynEdit.CaretY, TmpSynEdit.CaretX]);

  if (scInsertMode in Changes) or (scAll in Changes)then begin
    if TmpSynEdit.InsertMode then begin
      StatusBar.Panels[2].Text := 'Insert';
    end else begin
      StatusBar.Panels[2].Text := 'Overwrite';
    end;
  end;

  if (scModified in Changes) or (scAll in Changes) then begin
    if TmpSynEdit.Modified then begin
      StatusBar.Panels[1].Text := 'Modified';
    end else begin
      StatusBar.Panels[1].Text := '';
    end;
  end;
  StatusBar.Invalidate;

end;

procedure TFXMLEdit.PageControlXMLChange(Sender: TObject);
var TmpSynEdit: TSynEdit;
begin
  TmpSynEdit := GetSynEdit();
  if TmpSynEdit = nil then exit;

  TmpSynEdit.OnStatusChange(self, [scAll]);
end;

procedure TFXMLEdit.MenuUndoClick(Sender: TObject);
begin
  ActSynEdit.Undo;
end;

procedure TFXMLEdit.MenuRedoClick(Sender: TObject);
begin
  ActSynEdit.Redo;
end;

procedure TFXMLEdit.MenuFindClick(Sender: TObject);
begin
  FindDialog.Execute;
end;

procedure TFXMLEdit.FindDialogFind(Sender: TObject);
var TmpSynEdit: TSynEdit;
begin
  TmpSynEdit := GetSynEdit();
  if TmpSynEdit = nil then exit;

  FindReplaceDialog(TmpSynEdit, FindDialog);
end;

procedure TFXMLEdit.MenuReplaceClick(Sender: TObject);
begin
  ReplaceDialog.Execute;
end;

//  TFindOption = (frDown, frFindNext, frHideMatchCase, frHideWholeWord,
//    frHideUpDown, frMatchCase, frDisableMatchCase, frDisableUpDown,
//    frDisableWholeWord, frReplace, frReplaceAll, frWholeWord, frShowHelp);

//  TSynSearchOption = (ssoMatchCase, ssoWholeWord, ssoBackwards,
//    ssoEntireScope, ssoSelectedOnly, ssoReplace, ssoReplaceAll, ssoPrompt);

procedure TFXMLEdit.FindReplaceDialog(TmpSynEdit: TSynEdit; Dialog: TFindDialog);
var SynSearchOptions: TSynSearchOptions;
    replace_txt: string;
begin
  TmpSynEdit.SearchEngine := SynEditSearch;
  SynSearchOptions := [];
  if not (frDown in Dialog.Options) then
    SynSearchOptions := SynSearchOptions + [ssoBackwards];

  if (frReplace in Dialog.Options) then
    SynSearchOptions := SynSearchOptions + [ssoReplace];
  if (frReplaceAll in Dialog.Options) then
    SynSearchOptions := SynSearchOptions + [ssoReplaceAll];

  if (frWholeWord in Dialog.Options) then
    SynSearchOptions := SynSearchOptions + [ssoWholeWord];
  if (frMatchCase in Dialog.Options) then
    SynSearchOptions := SynSearchOptions + [ssoMatchCase];

  if Dialog is TReplaceDialog then begin
    replace_txt := TReplaceDialog(Dialog).ReplaceText;
    // Just to make the next Replace hit the selected word by find
    if [ssoReplace, ssoReplaceAll] * SynSearchOptions <> [] then
      SynSearchOptions := SynSearchOptions + [ssoSelectedOnly];
  end else begin
    replace_txt := '';
  end;
  TmpSynEdit.SearchReplace(Dialog.FindText, replace_txt, SynSearchOptions)
end;


procedure TFXMLEdit.ReplaceDialogAction(Sender: TObject);
var TmpSynEdit: TSynEdit;
begin
  TmpSynEdit := GetSynEdit();
  if TmpSynEdit = nil then exit;

  FindReplaceDialog(TmpSynEdit, ReplaceDialog);
end;

procedure TFXMLEdit.MenuSaveClick(Sender: TObject);
var i: integer;
    TmpSynEdit: TSynEdit;
begin
  i := PageControlXML.TabIndex;
  if (WorldODE.XMLFiles.Objects[i] is TSynEdit) then begin
    TmpSynEdit := TSynEdit(WorldODE.XMLFiles.Objects[i]);
  end else exit;

  TmpSynEdit.Lines.SaveToFile(WorldODE.XMLFiles[i]);
  TmpSynEdit.Modified := false;
end;

procedure TFXMLEdit.MenuSaveAsClick(Sender: TObject);
var i: integer;
    TmpSynEdit: TSynEdit;
begin
  i := PageControlXML.TabIndex;
  if (WorldODE.XMLFiles.Objects[i] is TSynEdit) then begin
    TmpSynEdit := TSynEdit(WorldODE.XMLFiles.Objects[i]);
  end else exit;

  if SaveDialog.initialDir ='' then SaveDialog.initialDir := GetCurrentDir;
  SaveDialog.FileName := WorldODE.XMLFiles[i];
  if not SaveDialog.Execute then exit;

  WorldODE.XMLFiles[i] := ExtractFileName(SaveDialog.FileName);
  (TmpSynEdit.Parent as TTabSheet).Caption := WorldODE.XMLFiles[i];
  TmpSynEdit.Lines.SaveToFile(WorldODE.XMLFiles[i]);
  TmpSynEdit.Modified := false;
end;


procedure TFXMLEdit.MenuNewClick(Sender: TObject);
var i: integer;
    sname: string;
    SL: TStringlist;
begin
  //i := PageControlXML.TabIndex;
  i := WorldODE.XMLFiles.Count;
  sname := 'untitled.xml';
  while true do begin
    if not InputQuery('Select file Name', 'New File to be created:', sname) then exit;
    if fileexists(sname) then begin
      Showmessage('File: '+ sname + ' already exists');
      continue;
    end;
    SL := TStringlist.Create;
    try
      SL.Add('<?xml version="1.0" ?>');
      SL.SaveToFile(sname);
    finally
      SL.Free;
    end;
    break;
  end;
  WorldODE.XMLFiles.Insert(i, sname);
  CreateXMLTabEdit(WorldODE.XMLFiles, i);
end;

procedure TFXMLEdit.MenuOpenClick(Sender: TObject);
var i: integer;
    sname: string;
begin
  if OpenDialog.initialDir ='' then OpenDialog.initialDir := GetCurrentDir;
  if not OpenDialog.Execute then exit;

  sname := ExtractFileName(OpenDialog.FileName);
  if not fileexists(sName) then exit; // TODO: queixar ao utilizador

  i := WorldODE.XMLFiles.Count;
  WorldODE.XMLFiles.Insert(i, sname);
  CreateXMLTabEdit(WorldODE.XMLFiles, i);

end;


procedure TFXMLEdit.MenuExitClick(Sender: TObject);
var i: integer;
    TmpSynEdit: TSynEdit;
    AnyModified: boolean;
begin
  AnyModified := false;
  for i := 0 to WorldODE.XMLFiles.Count -1 do begin
    if (WorldODE.XMLFiles.Objects[i] is TSynEdit) then begin
      TmpSynEdit := TSynEdit(WorldODE.XMLFiles.Objects[i]);
    end else continue;
    AnyModified := AnyModified or TmpSynEdit.Modified;
  end;

  if AnyModified then begin
    if MessageDlg('SomeFiles were changed.'+ #13 + #10 +
                  'Exit anyway?',
                  mtConfirmation , [mbOk,mbCancel], 0)
       = mrCancel then exit;
  end;
  FViewer.close;
end;

procedure TFXMLEdit.FormCreate(Sender: TObject);
begin
  FormStorage.IniFileName := GetIniFineName;
end;

end.