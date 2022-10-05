#include "stlastr.h"

#include <iostream>
#include <stdio.h>
#include <math.h>

using namespace std;

const int MAP_WIDTH = 20;
const int MAP_HEIGHT = 20;

int world_map[ MAP_WIDTH * MAP_HEIGHT ] = 
{
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,   // 00
	1,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,1,   // 01
	1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 02
	1,9,9,1,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 03
	1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,   // 04
	1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,   // 05
	1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,   // 06
	1,9,9,9,9,9,9,9,9,1,1,1,9,9,9,9,9,9,9,1,   // 07
	1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,   // 08
	1,9,1,9,9,9,9,9,9,9,1,1,9,9,9,9,9,9,9,1,   // 09
	1,9,1,1,1,1,9,1,1,9,1,1,1,1,1,1,1,1,1,1,   // 10
	1,9,9,9,9,9,1,9,1,9,1,9,9,9,9,9,1,1,1,1,   // 11
	1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 12
	1,9,1,9,1,9,9,9,1,9,1,9,1,9,1,9,9,9,1,1,   // 13
	1,9,1,1,1,1,9,9,1,9,1,9,1,1,1,1,9,9,1,1,   // 14
	1,9,1,1,9,1,1,1,1,9,1,1,1,1,9,1,1,1,1,1,   // 15
	1,9,9,9,9,1,1,1,1,1,1,9,9,9,9,1,1,1,1,1,   // 16
	1,1,9,9,9,9,9,9,9,1,1,1,9,9,9,1,9,9,9,9,   // 17
	1,9,1,1,1,1,1,1,1,1,1,9,1,1,1,1,1,1,1,1,   // 18
	1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,   // 19

};

int way[8]={0,-1,0,1,-1,0,1,0};

// 다음 단계로 갈 수 있는 좌표인지 확인
int GetMap(int x, int y){
    if ( x<0 || x>=MAP_WIDTH || y<0 || y>=MAP_HEIGHT){
        return 0;
    }
    return world_map[(y*MAP_WIDTH)+x]; // 벽인지 아닌지 체크해서 줌
}

class MapSearchNode{
    public:
        int x;
        int y;

        MapSearchNode() {x=y=0;}
        MapSearchNode(int px, int py){x=px; y=py;}

        float GoalDistanceEstimate(MapSearchNode &nodeGoal);
        bool IsGoal(MapSearchNode &nodeGoal);
        bool GetSuccessors(AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node);
        float GetCost(MapSearchNode &successor);
        bool IsSameState(MapSearchNode &rhs);

        void PrintNodeInfo();
};

bool MapSearchNode::IsSameState(MapSearchNode &rhs){
    if ((x == rhs.x) && (y==rhs.y)){
        return true;
    }else{
        return false;
    }
}

void MapSearchNode::PrintNodeInfo(){
    char str[100];
    sprintf(str,"Node position: (%d,%d)\n",x,y);

    cout << str;
}

//Huristic function
float MapSearchNode::GoalDistanceEstimate(MapSearchNode &nodeGoal){
    return abs(x - nodeGoal.x) + abs(y - nodeGoal.y);
}

bool MapSearchNode::IsGoal(MapSearchNode &nodeGoal){
    if((x==nodeGoal.x)&&(y==nodeGoal.y)){
        return true;
    }
    return false;
}

bool MapSearchNode::GetSuccessors(AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node){
    int parent_x = -1;
    int parent_y = -1;

    if(parent_node){
        parent_x = parent_node->x;
        parent_y = parent_node->y;
    }

    MapSearchNode NewNode;

    // 갈 수 있는 노드들을 넣기
    // Map이 벽인지 아닌지 확인 && parent랑 좌표가 같은지 확인
    for (int i=0; i<8; i+=2){
        int next_x = x + way[i];
        int next_y = y + way[i+1];
        if((GetMap(next_x,next_y)<9) && !((parent_x==next_x)&&(parent_y==next_y))){
            NewNode = MapSearchNode(next_x,next_y);
            astarsearch->AddSuccessor(NewNode);
        }
    }
    return true;
}

float MapSearchNode::GetCost(MapSearchNode &successor){
    return (float) GetMap(x,y);
}

int main(int argc, char *argv[]){
    cout << "STD A* Search implementation\n";

    AStarSearch<MapSearchNode> astarsearch;

    unsigned int SearchCount = 0;

    const unsigned int NumSearches = 1;
    
    while(SearchCount < NumSearches){

        //Create a start state
        MapSearchNode nodeStart;
        nodeStart.x = rand()%MAP_WIDTH;
        nodeStart.y = rand()%MAP_HEIGHT;

        //Define the goal state
        MapSearchNode nodeEnd;
        nodeEnd.x = rand()%MAP_WIDTH;
        nodeEnd.y = rand()%MAP_HEIGHT;

        // set start and goal state
        astarsearch.SetStartAndGoalStates(nodeStart,nodeEnd);

        unsigned int SearchState;
        unsigned int SearchSteps = 0;

        do{
            SearchState = astarsearch.SearchStep();
            SearchSteps++;
        }while(SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING);

        if(SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED){
            cout << "Search found goal state \n";
            MapSearchNode *node = astarsearch.GetSolutionStart();

            int steps = 0;

            node -> PrintNodeInfo();
            for(;;){
                node = astarsearch.GetSolutionNext();
                if(!node){break;}

                node->PrintNodeInfo();
                steps++;
            };

            cout << "Solution steps" << steps << endl;
        }else if(SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED){
            cout << "Search terminated. 목적지를 찾을 수 없음" << endl;
        }

        cout << "SearchSteps : "<< SearchSteps << endl;

        SearchCount++;

        // astarsearch.EnsureMemoryFreed();

    }

    return 0;
    
}
