#include <chrono>
#include <iostream>
#include "sFoundation/pubSysCls.h"	

using namespace sFnd;

int main(int /*argc*/, char** /*argv[]*/)
{
  SysManager* myMgr = SysManager::Instance();	
  try
  {
    myMgr->ComHubPort(0, "/dev/ttyXRUSB0");
    myMgr->PortsOpen(1);
    INode &inode = myMgr->Ports(0).Nodes(0);

    // disable node
    inode.EnableReq(false);
    printf("Disabled node\n");
  }		
  catch(mnErr& theErr)
  {
    printf("Port Failed to open, Check to ensure correct Port number and that ClearView is not using the Port\n");	
    printf("Caught error: addr=%d, err=0x%08x\nmsg=%s\n", theErr.TheAddr, theErr.ErrorCode, theErr.ErrorMsg);    
    return 0;
  }

  myMgr->PortsClose();
  return 0;
}
