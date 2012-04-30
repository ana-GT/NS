/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved	
 * Author(s): Ana C. Huaman Quispe <ahuaman3@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */	

#ifndef _NS_TAB_
#define _NS_TAB_

#include <Tabs/GRIPTab.h>
#include <Tabs/GRIPThread.h>

#include <planning/Robot.h>
#include <planning/Object.h>
#include <kinematics/BodyNode.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/Transformation.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>

#include "myFunctions/CheckProcess.h"

#include <iostream>
#include <list>

using namespace std;

/**
 * @class NS
 * @brief Tab for NullSpace tests
 */
class NSTab : public GRIPTab
{
public:

    /// Variables to save info for the planners
    Eigen::VectorXd mCurrentConfig;
    Eigen::VectorXd mCurrentPos; /**< Dim: 3 */

    Eigen::MatrixXd mJaclin;
    Eigen::MatrixXd mNS_Basis;
    Eigen::MatrixXd mNS_Coeff;

    //-- Robot specific info
    int mRobotId;
    int mEEId; /**< End Effector ID */
    kinematics::BodyNode *mEENode;
    std::string mEEName;
    Eigen::VectorXi mLinks;

    int mNS_Dim;
  
    /// Miscellaneous stuff ( no idea why this is here )
    wxTextCtrl *timeText;

    /// Public vars to capture external selection stuff 
    planning::Object* selectedObject;
    planning::Robot* selectedRobot;
    kinematics::BodyNode* selectedNode;
    
    /// Functions about Robina's left arm specifically
    Eigen::VectorXi GetLeftArmIds();
    
    /// Functions related to Tab
    NSTab(){};
    NSTab( wxWindow * parent, wxWindowID id = -1,
	   const wxPoint & pos = wxDefaultPosition,
	   const wxSize & size = wxDefaultSize,
	   long style = wxTAB_TRAVERSAL);
    virtual ~NSTab();
    
    void OnSlider(wxCommandEvent &evt);
    void OnRadio(wxCommandEvent &evt);
    void OnButton(wxCommandEvent &evt);
    void OnCheckBox(wxCommandEvent &evt);
    
    //-- Workspace functions
    void SetCurrentConfig();
    Eigen::VectorXd GetEE_XYZ( const Eigen::VectorXd &_q );
    void RunOnSliderChange( int _coeff, double _value );
    
    void SetTimeline( std::vector<Eigen::VectorXd> _path );
    void GRIPStateChange();
    
    //-- Manipulate the nullspace vectors
    GRIPSlider* mNS[4];

    DECLARE_DYNAMIC_CLASS( NSTab )
    DECLARE_EVENT_TABLE()
};

#endif /** _NS_TAB_ */

