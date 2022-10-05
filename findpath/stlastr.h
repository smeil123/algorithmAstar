#ifndef STLASTAR_H
#define STLASTAR_H
//text 디버깅용
#include <iostream>
#include <stdio.h>
#include <assert.h>

#include <algorithm>
#include <set>
#include <vector>
#include <cfloat>

using namespace std;

// //메모리 관리용
// #include "fsa.h"

// #define USE_FAS_MEMORY 1

// #if defined(WIN)

//Astar Search class
template <class UserState> class AStarSearch{
    public:
        enum{
            SEARCH_STATE_NOT_INITALISED,
            SEARCH_STATE_SEARCHING,
            SEARCH_STATE_SUCCEEDED,
            SEARCH_STATE_FAILED,
            SEARCH_STATE_OUT_OF_MEMORY,
            SEARCH_STATE_INVALID
        };

    public:
        class Node{
            public:
                Node *parent;
                Node *child;

                float g; //이동거리
                float h; //휴리스틱으로 계산, 목적지까지의 비용
                float f; //g+h

                Node():
                    parent(0),
                    child(0),
                    g(0.0f),
                    h(0.0f),
                    f(0.0f)
                {

                }

                UserState m_UserState;
        };

        class HeapCompare_f{
            public:
                bool operator() (const Node *x, const Node *y) const{
                    return x->f > y->f;
                }
        };

    public: //method
        AStarSearch():
            m_State(SEARCH_STATE_NOT_INITALISED),
            m_CurrentSolutionNode(NULL),
            m_AllocateNodeCount(0),
            m_CancelRequest(false)

        {

        }
        AStarSearch(int MaxNodes) : 
            m_State(SEARCH_STATE_NOT_INITALISED),
            m_CurrentSolutionNode(NULL),
            m_AllocateNodeCount(0),
            m_CancelRequest(false)
        {

        }
        
        void CancelSearch(){
            m_CancelRequest = true;
        }

        // Set Start, Goal
        void SetStartAndGoalStates(UserState &Start, UserState &Goal){
            m_CancelRequest = false;

            m_Start = AllocateNode();
            m_Goal = AllocateNode();

            assert((m_Start != NULL && m_Goal != NULL));

            m_Start->m_UserState = Start;
            m_Goal->m_UserState = Goal;

            m_State = SEARCH_STATE_SEARCHING;

            //Start Node INIT
            m_Start->g = 0;
            m_Start->h = m_Start->m_UserState.GoalDistanceEstimate(m_Goal->m_UserState);
            m_Start->f = m_Start->g + m_Start->h;
            m_Start->parent = 0;

            //Open List 에 Start Node push
            m_OpenList.push_back(m_Start);

            push_heap(m_OpenList.begin(), m_OpenList.end(), HeapCompare_f());

            m_Steps = 0;
        }

        unsigned int SearchStep(){
            // init 상태인지 체크
            assert((m_State > SEARCH_STATE_NOT_INITALISED) && (m_State < SEARCH_STATE_INVALID));

            // 이미 결과가 나온게 아닌지 한번 더 확인
            if((m_State == SEARCH_STATE_SUCCEEDED) || (m_State == SEARCH_STATE_FAILED)){
                return m_State ;
            }
            // open_list 가 비어있는지 확인
            if(m_OpenList.empty() || m_CancelRequest){
                FreeAllNodes();
                m_State = SEARCH_STATE_FAILED;
                return m_State;
            }
            m_Steps ++;

            // Pop
            // ?? 왜 이렇게 가져가는거지?
            Node *n = m_OpenList.front();
            pop_heap(m_OpenList.begin(), m_OpenList.end(), HeapCompare_f());
            m_OpenList.pop_back();

            //check goal
            if(n->m_UserState.IsGoal(m_Goal->m_UserState)){
                //목표에 도착한 경우, parent 와 g를 복사
                m_Goal->parent = n->parent;
                m_Goal->g = n->g;

                //특수한 경우, start == goal
                if(false == n->m_UserState.IsSameState(m_Start->m_UserState)){
                    FreeNode(n);

                    // ??
                    Node *nodeChild = m_Goal;
                    Node *nodeParent = m_Goal->parent;

                    do{
                        nodeParent->child = nodeChild;

                        nodeChild = nodeParent;
                        nodeParent = nodeParent->parent;
                    }while(nodeChild != m_Start); //Start는 항상 첫번째 노드로 정의됨
                }

                FreeUnusedNodes();

                m_State = SEARCH_STATE_SUCCEEDED;

                return m_State;
            }
            else{//not goal
                //
                m_Successors.clear();
                
                //갈 수 있는 다음길을 찾음
                //parent ?? or null
                bool ret = n->m_UserState.GetSuccessors(this, n->parent ? &n->parent->m_UserState : NULL);

                if(!ret){
                    //next길이 있으면
                    typename vector<Node *>::iterator successor;

                    //혹시나 이전에 clear되지 않았던게 있을 수 있어 clear해줌
                    for(successor = m_Successors.begin(); successor != m_Successors.end(); successor++){
                        FreeNode( (*successor));
                    }
                    
                    m_Successors.clear();

                    FreeNode((n));
                    FreeAllNodes();

                    m_State = SEARCH_STATE_OUT_OF_MEMORY;
                    return m_State;

                }

                //next node를 하나씩 보기
                for(typename vector<Node *>::iterator successor = m_Successors.begin(); successor!=m_Successors.end();successor++){
                    float newg = n->g + n->m_UserState.GetCost((*successor)->m_UserState);

                    typename vector<Node *>::iterator openlist_result;

                    // 이미 openlist에 있는 노드인지 확인
                    for(openlist_result = m_OpenList.begin(); openlist_result != m_OpenList.end(); openlist_result++){
                        if((*openlist_result)->m_UserState.IsSameState((*successor)->m_UserState)){
                            break;
                        }
                    }
                    if(openlist_result != m_OpenList.end()){
                        //이미 openlist에 있는게 cost가 더 낮으면 현재 노드는 pass
                        if( (*openlist_result)->g <= newg){
                            FreeNode((*successor));
                            continue;
                        }
                    }

                    typename vector<Node *>::iterator closedlist_result;
                    for(closedlist_result = m_ClosedList.begin(); closedlist_result != m_ClosedList.end(); closedlist_result++){
                        if((*closedlist_result)->m_UserState.IsSameState((*successor)->m_UserState)){
                            break;
                        }
                    }
                    if(closedlist_result != m_ClosedList.end()){
                        //이미 closedlist에 있는게 cost가 더 낮으면 현재 노드는 pass
                        if((*closedlist_result)->g <= newg){
                            FreeNode((*successor));
                            continue;
                        }
                    }

                    (*successor)->parent = n;
                    (*successor)->g = newg;
                    (*successor)->h = (*successor)->m_UserState.GoalDistanceEstimate(m_Goal->m_UserState);
                    (*successor)->f = (*successor)->g + (*successor)->h;

                    // 1. update old version of this node in closed list
                    // 2. move it from closed to open_list
                    // 3. sort heal again in open list

                    //openlist - 갈 수 있는 모든 리스트
                    //closedlist - openlist 중 가장 목적지까지 가까운 노드

                    if(closedlist_result != m_ClosedList.end()){ 
                        //노드가 closed_list에 있었다면
                        //closed_list update
                        (*closedlist_result)->parent = (*successor)->parent;
                        (*closedlist_result)->g = (*successor)->g;
                        (*closedlist_result)->h = (*successor)->h;
                        (*closedlist_result)->f = (*successor)->f;

                        FreeNode((*successor));

                        //왜 seccessor을 안넣고 이렇게하는걸까?
                        m_OpenList.push_back((*closedlist_result));

                        m_ClosedList.erase(closedlist_result);

                        push_heap(m_OpenList.begin(),m_OpenList.end(), HeapCompare_f());
                    }

                    //1. update old version of this node in open list
                    //2. sort heap again in open list

                    else if(openlist_result != m_OpenList.end()){
                        (*openlist_result)->parent = (*successor)->parent;
                        (*openlist_result)->g = (*successor)->g;
                        (*openlist_result)->h = (*successor)->h;
                        (*openlist_result)->f = (*successor)->f;

                        FreeNode((*successor));

                        make_heap(m_OpenList.begin(), m_OpenList.end(),HeapCompare_f());
                    }

                    // 새로운 노드이면
                    //1. move it from successors to open list
                    //2. sort heal again in open list
                    else{
                        m_OpenList.push_back((*successor));

                        push_heap(m_OpenList.begin(),m_OpenList.end(), HeapCompare_f());
                    }
                }

                m_ClosedList.push_back(n);
            }

            return m_State;
        } 

        bool AddSuccessor(UserState &State){
            Node *node = AllocateNode();

            if(node){
                node -> m_UserState = State;
                m_Successors.push_back(node);
                return true;
            }
            return false;
        }
        // 모든 노드 free
        void FreeSolutionNodes(){
            Node *n = m_Start;
            if(m_Start ->child){
                do{
                    Node *del = n;
                    n = n->child;
                    FreeNode(del);

                    del=NULL;
                }while(n!=m_Goal);
                FreeNode(n);
            }
            else{
                FreeNode(m_Start);
                FreeNode(m_Goal);
            }
        }

        //Get Start node
        UserState *GetSolutionStart(){
            m_CurrentSolutionNode = m_Start;
            if(m_Start){
                return &m_Start->m_UserState;
            }
            else{
                return NULL;
            }
        }

        //Get Next node(child)
        UserState *GetSolutionNext(){
            if(m_CurrentSolutionNode){
                if(m_CurrentSolutionNode->child){
                    Node *child = m_CurrentSolutionNode->child;
                    m_CurrentSolutionNode = m_CurrentSolutionNode->child;
                    return &child->m_UserState;
                }
            }

            return NULL;
        }

        //Get End node
        UserState *GetSolutionEnd(){
            m_CurrentSolutionNode = m_Goal;
            if(m_Goal){
                return &m_Goal->m_UserState;
            }else{
                return NULL;
            }
        }

        //Get Backwards
        UserState *GetSolutionPrev(){
            if(m_CurrentSolutionNode){
                if(m_CurrentSolutionNode->parent){
                    Node *parent = m_CurrentSolutionNode->parent;
                    m_CurrentSolutionNode = m_CurrentSolutionNode->parent;
                    return &parent->m_UserState;
                }
            }
            return NULL;
        }

        float GetSolutionCost(){
            if(m_Goal && m_State == SEARCH_STATE_SUCCEEDED){
                return m_Goal->g;
            }else{
                return FLT_MAX;
            }
        }

        UserState *GetOpenListStart(){
            float f,g,h;
            return GetOpenListStart(f,g,h);
        }

        UserState *GetOpenListStart(float &f, float &g, float &h){
            iterDbgOpen = m_OpenList.begin();
            if(iterDbgOpen != m_OpenList.end()){
                f = (*iterDbgOpen)->f;
                g = (*iterDbgOpen)->g;
                h = (*iterDbgOpen)->h;
                return &(*iterDbgOpen)->m_UserState;
            }
            return NULL;
        }

        int GetStepCount(){return m_Steps;}
            
    private: //method

        void FreeAllNodes(){
            typename vector<Node *>::iterator iterOpen = m_OpenList.begin();

            while(iterOpen != m_OpenList.end()){
                Node *n = (*iterOpen);
                FreeNode(n);

                iterOpen++;
            }

            m_OpenList.clear();

            typename vector<Node *>::iterator iterCloed = m_ClosedList.begin();

            while(iterCloed != m_ClosedList.end()){
                Node *n = (*iterCloed);
                FreeNode(n);

                iterCloed++;
            }

            m_ClosedList.clear();

            FreeNode(m_Goal);
        }

        //검색이 종료될때 수행(목적지에 도착했을때)
        void FreeUnusedNodes(){
            typename vector<Node *>::iterator iterOpen= m_OpenList.begin();
            while(iterOpen != m_OpenList.end()){
                Node *n = (*iterOpen);

                if(!n->child){
                    FreeNode(n);
                    n = NULL;
                }

                iterOpen++;
            }
            // iterate closed list and delete unused nodes
		    typename vector< Node * >::iterator iterClosed;

	    	for( iterClosed = m_ClosedList.begin(); iterClosed != m_ClosedList.end(); iterClosed ++ ){
	    		Node *n = (*iterClosed);

	    		if( !n->child ){
	    			FreeNode( n );
		    		n = NULL;
		    	}
		    }

		    m_ClosedList.clear();
        }

        Node *AllocateNode(){
            m_AllocateNodeCount++;
            Node *p = new Node;
            return p;
        }

        void FreeNode(Node *node){
            m_AllocateNodeCount--;
            delete node;
        }
    
    private: //data
        vector<Node *> m_OpenList;
        vector<Node *> m_ClosedList;
        vector<Node *> m_Successors;

        unsigned int m_State;
        int m_Steps;

        Node *m_Start;
        Node *m_Goal;

        Node *m_CurrentSolutionNode;

        typename vector<Node *>::iterator iterDbgOpen;        
        typename vector<Node *>::iterator iterDbgClosed;

        int m_AllocateNodeCount;

        bool m_CancelRequest;        

};

template <class T> class AStarState{
    public:
        virtual ~AStarState(){}
        virtual float GoalDistanceEstimate(T &nodeGoal) = 0; // heuristic 계산
        virtual bool IsGoal( T &nodeGoal ) = 0; // 도착했는지 혹인
	    virtual bool GetSuccessors( AStarSearch<T> *astarsearch, T *parent_node ) = 0; // 다음 스텝으로 갈 수 있는 노드를 찾음 그 다음 addSuccessor()
	    virtual float GetCost( T &successor ) = 0; // Computes the cost of travelling from this node to the successor node
	    virtual bool IsSameState( T &rhs ) = 0; // Returns true if this node is the same as the rhs node
};


#endif