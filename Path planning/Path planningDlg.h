
// Path planningDlg.h : 標頭檔
//

#pragma once
#include "afxwin.h"
#include "cv.h"
#include "highgui.h"
#include "CvvImage.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
using namespace cv;
using namespace std;

struct draw_car
{
	CvPoint car[6];
	vector <CvPoint>sim_path;
};

// CPathplanningDlg 對話方塊
class CPathplanningDlg : public CDialogEx
{
	// 建構
public:
	CPathplanningDlg(CWnd* pParent = NULL);	// 標準建構函式

// 對話方塊資料
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_PATHPLANNING_DIALOG };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支援


// 程式碼實作
protected:
	HICON m_hIcon;

	// 產生的訊息對應函式
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButtonStart();

	void find_path(int x);
	void dijkstra(int source, int node_num);
	void binarization(IplImage *i_show_data, vector<vector<bool>>  &o_sca_image2);
	void find_coner(vector<vector<bool>> i_sca_image, vector <Point> &o_save_coner, int i_Interpolation);

	void trans2Voronoi(vector<vector<bool>> i_sca_image, vector <Point> i_save_coner, double(&o_Data)[8000], int i_Interpolation2);

	void Voronoi_calculate(double i_Data[8000], int x_boundary, int y_boundary, CvPoint2D64f(&o_savepoint1)[3000], CvPoint2D64f(&o_savepoint2)[3000], int &o_line_count);

	void Generalized_Voronoi(vector<vector<bool>> i_sca_image, CvPoint2D64f i_savepoint1[3000], CvPoint2D64f i_savepoint2[3000], int i_line_count, int &o_new_input_index, CvPoint2D64f(&o_new_savepoint1)[3000], CvPoint2D64f(&o_new_savepoint2)[3000]);

	void Match_point(int i_line_count, int i_new_input_index, CvPoint2D64f(&io_new_savepoint1)[3000], CvPoint2D64f(&io_new_savepoint2)[3000], float near_dis);

	void Dijkstra_path_planning(CvPoint i_robot_start, CvPoint  i_robot_end, CvPoint2D64f i_new_savepoint1[3000], CvPoint2D64f i_new_savepoint2[3000], int i_new_input_index, vector <CPoint> &o_all_point_map, vector <CvPoint2D64f> &o_all_point_map_original);

	void Path_Optimization(vector<vector<bool>> i_sca_image, vector <CvPoint2D64f> i_all_point_map_original, vector <int> &o_path_optimization);

	void Path_simulation(vector <CPoint> i_path, int i_car_density, vector <CPoint> &o_sim_path, vector <draw_car> &o_sim_car, IplImage *&draw_data); //輸入路徑、車子間距，輸出模擬路徑、車子

	void Control_Methods(bool control_type, double i_rho, double i_alpha, double i_beta, double i_phi, double &o_vr, double &o_vl, int &o_state);
	CStatic m_show;
	int m_coner_count;
	int m_total_time;
	CStatic m_show2;
};


class float_point
{
	float x;
	float y;
};

class CElement : public CObject
{
	DECLARE_SERIAL(CElement)

protected:
	COLORREF m_Color;                       // Color of an element
	CRect m_EnclosingRect;                  // Rectangle enclosing an element
	int m_Pen;                              // Pen width

public:
	virtual ~CElement() {}                   // Virtual destructor

											 // Virtual draw operation
	virtual void Draw(CDC* pDC, const CElement* pElement = 0) const {}
	virtual void Move(const CSize& Size) {} // Move an element
	CRect GetBoundRect() const;             // Get the bounding rectangle for an element

	virtual void Serialize(CArchive& ar);      // Serialize function for CElement

protected:
	CElement() {}                            // Default constructor
};


// Class defining a vor_point object
class CVor_Point : public CElement
{
	DECLARE_SERIAL(CVor_Point)

public:
	int GetPositionY();
	int GetPositionX();
	// Function to display a line
	virtual void Draw(CDC* pDC, const CElement* pElement = 0) const;
	virtual void Move(const CSize& aSize);       // Function to move an element

												 // Constructor for a line object
	CVor_Point(const CPoint& Start, const COLORREF& Color, const int& PenWidth);

	virtual void Serialize(CArchive& ar);      // Serialize function for CVor_Point

protected:
	CPoint m_StartPoint;          // Start point of line

	CVor_Point() {}             // Default constructor - should not be used
};

class SK_point
{
public:
	int x;
	int y;
	float distant;
};

