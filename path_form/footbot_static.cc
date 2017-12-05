/////////////////////////////////
// File: footbot_static.cc
// Desc: nest and prey robot for path formation
// Created: 2016.3.17
// Author: Tetsuya Idota
// License: GPL
/////////////////////////////////

//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>

#include "stage.hh"
using namespace Stg;

// Stage calls this when the model starts up
extern "C" int Init( Model* mod )
{
  ModelPosition* position = (ModelPosition*)mod->GetUnusedModelOfType( "position" );

  position->Subscribe();
  position->SetFiducialReturn(0);
  
  
  ModelPosition* marker = (ModelPosition*)mod->GetUnusedModelOfType( "position" ) ;  
  if(marker != NULL){
    marker->Subscribe();
    marker->SetColor(mod->GetColor());
    marker->SetFiducialReturn(1);
  }

  return 0; //ok
}

