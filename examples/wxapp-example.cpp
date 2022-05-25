// wxWidgets "Hello world" Program
// For compilers that support precompilation, includes "wx/wx.h".
#include <wx/wx.h>
#include <wx/sizer.h>

/**
 * @brief Enumeration for custom events for later use
 */
enum
{
    ID_Hello = 1,
};

/**
 * @brief Specialized frame of the "My" type
 */
class MyFrame: public wxFrame
{
public:
    MyFrame(const wxString& title, const wxPoint& pos, const wxSize& size);
private:
    void OnHello(wxCommandEvent& event);
    void OnExit(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);
    wxDECLARE_EVENT_TABLE();
};

/**
 * @brief Construct a new My Frame:: My Frame object
 * 
 * @param title Title to put on top of the frame
 * @param pos The position to put the window
 * @param size The initial size to make the box
 */
MyFrame::MyFrame(const wxString& title, const wxPoint& pos, const wxSize& size)
        : wxFrame(NULL, wxID_ANY, title, pos, size)
{
    wxMenu *menuFile = new wxMenu;
    menuFile->Append(ID_Hello, "&Hello...\tCtrl-H",
                     "Help string shown in status bar for this menu item");
    menuFile->AppendSeparator();
    menuFile->Append(wxID_EXIT);
    wxMenu *menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT);
    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append( menuFile, "&File" );
    menuBar->Append( menuHelp, "&Help" );
    SetMenuBar( menuBar );
    CreateStatusBar();
    SetStatusText( "Welcome to wxWidgets!" );
}

/**
 * @brief MyFrame callback for exiting
 * 
 * @param event The event object captured
 */
void MyFrame::OnExit(wxCommandEvent& event)
{
    Close( true );
}

/**
 * @brief MyFrame callback for clicking on "About"
 * 
 * @param event The event object captured
 */
void MyFrame::OnAbout(wxCommandEvent& event)
{
    wxMessageBox( "This is a wxWidgets' Hello world sample",
                  "About Hello World", wxOK | wxICON_INFORMATION );
}

/**
 * @brief MyFrame callback for clicking on "Hello"
 * 
 * @param event The event object captured
 */
void MyFrame::OnHello(wxCommandEvent& event)
{
    wxLogMessage("Hello world from wxWidgets!");
}

/**
 * @brief Construct a new wxBEGIN EVENT TABLE object
 * 
 * Create the MyFrame event table
 */
wxBEGIN_EVENT_TABLE(MyFrame, wxFrame)
    EVT_MENU(ID_Hello,   MyFrame::OnHello)
    EVT_MENU(wxID_EXIT,  MyFrame::OnExit)
    EVT_MENU(wxID_ABOUT, MyFrame::OnAbout)
wxEND_EVENT_TABLE()

/**
 * @brief The image panel
 */
class wxImagePanel : public wxPanel
    {
        wxBitmap image;

    public:
        wxImagePanel(wxFrame* parent, wxString file, wxBitmapType format);
    private:
        void paintEvent(wxPaintEvent & evt);
        void paintNow();
        void render(wxDC& dc);
        wxDECLARE_EVENT_TABLE();
    };

/**
 * @brief Construct a new wx Image Panel::wx Image Panel object
 * 
 * @param parent The wxFrame to which the Panel should belong
 * @param file The image file to load and display
 * @param format Image format to expect from the file
 */
wxImagePanel::wxImagePanel(wxFrame* parent, wxString file, wxBitmapType format) : wxPanel(parent)
{
    // load the file... ideally add a check to see if loading was successful
    image.LoadFile(file, format);
}

/**
 * @brief The paintEvent
 * 
 * @param evt The event object captured
 */
void wxImagePanel::paintEvent(wxPaintEvent & evt)
{
    // depending on your system you may need to look at double-buffered dcs
    wxPaintDC dc(this);
    render(dc);
}

void wxImagePanel::paintNow()
{
    // depending on your system you may need to look at double-buffered dcs
    wxClientDC dc(this);
    render(dc);
}

void wxImagePanel::render(wxDC&  dc)
{
    dc.DrawBitmap( image, 0, 0, false );
}

class MyApp: public wxApp
{
    MyFrame *frame;
    wxImagePanel *drawPane; 
    virtual bool OnInit();
};

wxBEGIN_EVENT_TABLE(wxImagePanel, wxPanel)
    EVT_PAINT(wxImagePanel::paintEvent)
wxEND_EVENT_TABLE()

bool MyApp::OnInit()
{
    wxInitAllImageHandlers();
    wxBoxSizer *sizer = new wxBoxSizer(wxHORIZONTAL);
    frame = new MyFrame("Hello World", wxPoint(50, 50), wxSize(800, 600));
    drawPane = new wxImagePanel(frame, wxT("detections.png"), wxBITMAP_TYPE_PNG);
    sizer->Add(drawPane, 1, wxEXPAND);
    frame->SetSizer(sizer);
    frame->Show( true );
    return true;
}

wxIMPLEMENT_APP(MyApp);

