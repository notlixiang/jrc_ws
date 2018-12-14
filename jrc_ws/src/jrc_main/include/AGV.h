//
// Created by sirius on 18-11-3.
//

#ifndef AGV_H
#define AGV_H

#include <map>
#include <vector>
#include <iostream>

using namespace std;
namespace jrc{
    class AGV {
    public:
        AGV();
        static int HOME;
        static int A;
        static int B;
        static int C;
        static int D;
        static int N_POSI;

        static int NOT_VISITED;
        static int VISITING;
        static int LEAVED;

        map<int, int> mAGVstatus;

        int getStatusAtPosition(const int& POSI);
        void setStatusAtPosition(const int& POSI, const int& STATUS);
        int getCurrentVisiting();
        int getCurrentHeading();
        void resetToStart();
        void resetToBreakPoint();

    };

    int AGV::HOME = 0;
    int AGV::A = 1;
    int AGV::B = 2;
    int AGV::C = 3;
    int AGV::D = 4;
    int AGV::N_POSI = 5;  // total 5 Positions

    int AGV::NOT_VISITED = 0;
    int AGV::VISITING = 1;
    int AGV::LEAVED = 2;
}


#endif //AGV_H
