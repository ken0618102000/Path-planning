
// Path planning.h : PROJECT_NAME ���ε{�����D�n���Y��
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�� PCH �]�t���ɮ׫e���]�t 'stdafx.h'"
#endif

#include "resource.h"		// �D�n�Ÿ�


// CPathplanningApp: 
// �аѾ\��@�����O�� Path planning.cpp
//

class CPathplanningApp : public CWinApp
{
public:
	CPathplanningApp();

// �мg
public:
	virtual BOOL InitInstance();

// �{���X��@

	DECLARE_MESSAGE_MAP()
};

extern CPathplanningApp theApp;