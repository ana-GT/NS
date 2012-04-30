 /*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

/**
 * @author A. Huaman
 * @date 2012-03-07
 */
#include "GRIPApp.h"
#include "NSTab.h"

extern wxNotebook* tabView;

class NSTabApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new NSTab(tabView), wxT("NS Tab"));
	}
};

IMPLEMENT_APP(NSTabApp)
