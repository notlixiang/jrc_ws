//
// Created by sirius on 18-11-3.
//

#include "AGV.h"

using namespace std;
namespace jrc{
    AGV::AGV() {
        for(int i  = 0; i < AGV::N_POSI; i++) {
            if(i == AGV::HOME) {
                mAGVstatus[AGV::HOME] = AGV::VISITING;
                continue;
            }
            mAGVstatus[i] = AGV::NOT_VISITED;
        }
    }

    int AGV::getStatusAtPosition(const int& position) {
        if (mAGVstatus.find(position) == mAGVstatus.end())  {
            cout << "当前AGV不具有此位置选项!" << endl;
            return -1;
        }
        return mAGVstatus[position];
    }

    void AGV::setStatusAtPosition(const int &POSI, const int &STATUS) {
        mAGVstatus[POSI] = STATUS;
    }

    int AGV::getCurrentVisiting() {
        // TODO 实现它
        return -1;
    }

    int AGV::getCurrentHeading() {
        // TODO 实现它
        return -1;
    }

    void AGV::resetToStart(){
        // TODO 实现它
    }

    void AGV::resetToBreakPoint(){
        // TODO 实现它
    }

}
