#include <stdio.h>
#include <malloc.h>
#include <assert.h>
#include <stdlib.h>
#include <time.h>

#define MAX_BOARD   25
#define MAX_DEPTH   200
#define ROW         5
#define COLUMN      5

struct board_s;

typedef struct board_s {
	struct board_s *pred;
	int f;
	int g;
	int h;
	char   array[MAX_BOARD];
	char   blank;
	int    depth;
} board_t;

/* Node Functions */

board_t *nodeAlloc(void) {
	board_t *board_p;

	board_p = (board_t *)malloc(sizeof(board_t));
	assert(board_p);

	board_p->pred = NULL;
	board_p->f = board_p->g = board_p->h = 0;

	return board_p;
}

void nodeFree(board_t *board_p) {
	assert(board_p);
	free(board_p);
	return;
}

#define MAX_LIST_ELEMENTS   1500000

typedef struct {
	int numElements;
	board_t *elements[MAX_LIST_ELEMENTS];
} list_t;

#define listCount(x)   ((x)->numElements)

list_t openList_p;
list_t closedList_p;
/*      list 초기화      */
void initList(list_t *list_p) {
	int i;

	assert(list_p);

	list_p->numElements = 0;

	for (i = 0; i < MAX_LIST_ELEMENTS; i++) 
		list_p->elements[i] = (board_t *)0;
	
	return;
}
/*list에 노드가 있는지 확인하며 위치를 return*/
int onList(list_t *list_p, char *board_p, int *pos){
	int i, j;

	assert(list_p);  assert(board_p);

	for (i = 0; i < list_p->numElements; i++) {
		
		if (list_p->elements[i] != (board_t *)0) {
			
			for (j = 0; j < MAX_BOARD; j++) {

				if (list_p->elements[i]->array[j] != board_p[j]) break;

			}

			if (j == MAX_BOARD) {
				//list에존재하면 어느 index에 존재하는지 알려주기위해 set
				if (pos) *pos = i;
				return 1;
			}
		}
	}
	//list에 없으면 0반환
	return 0;
}

/*h와 g값으로 가장 목표노드와 가까운 노드를 어림짐작으로 list에서 반환*/
board_t *getListBest(list_t *list_p){
	//list에서 우선순위가 가장 작은 노드를 뽑는다.
	int i;
	int best = -1;
	int best_f = 1000000;
	board_t *board_p;

	for (i = 0; i < list_p->numElements; i++){
		if (list_p->elements[i]){
			//f score가 작은것을 찾아서
			//best에 elements index를 넣는다
			//그 elements index의 f score를 best에 넣는다
			if (list_p->elements[i]->f < best_f) {                  
				best = i;                                    
				best_f = list_p->elements[best]->f;
			}
		}
	}
	assert(best != -1);

	board_p = list_p->elements[best];
	list_p->numElements--;//best elements를 꺼냈으므로 count 감소
	list_p->elements[best] = list_p->elements[list_p->numElements];//마지막 노드를 best에 넣고  
	list_p->elements[list_p->numElements] = NULL;//마지막 노드 삭제
	//best node return
	return board_p; 
}

/*list_t의 pos 위치에 있는 노드를 삭제하고 반환한다	*/
board_t *delList(list_t *list_p, int pos) {
	board_t *retboard_p;

	assert(list_p);

	retboard_p = list_p->elements[pos];
	list_p->numElements--;
	list_p->elements[pos] = list_p->elements[list_p->numElements];
	list_p->elements[list_p->numElements] = NULL;

	return retboard_p;
}

void putList(list_t *list_p, board_t *board_p) {
	assert(list_p); assert(board_p);

	if (list_p->numElements >= MAX_LIST_ELEMENTS) {
		//list 배열의 범위를 넘어간경우 fail
		printf("list is full (%d)\n", MAX_LIST_ELEMENTS);
		exit(1);
	}
	list_p->elements[list_p->numElements++] = board_p;
}

void cleanupList(list_t *list_p){
	int i;

	assert(list_p);

	for (i = 0; i < MAX_LIST_ELEMENTS; i++) {
		if (list_p->elements[i] != (board_t *)0) {
			nodeFree(list_p->elements[i]);
			list_p->numElements--;
		}
	}
	return;
}

/*      Hamming distance      */
int hamming(board_t *board_p) {
	int i;
	int score = 0;
	const int test[MAX_BOARD - 1] = { 
		1, 2, 3, 4, 5,
		6, 7, 8 ,9,10,
		11,12,13,14,15,
		16,17,18,19,20,
		21,22,23,24 };
	for (i = 0; i < MAX_BOARD - 1; i++)
		score += (board_p->array[i] != test[i]);

	return score;
}
/*         manhattan distance         */
int manhattan(board_t *board_p) {
	int i, index_x, index_y;
	int score = 0;//Manhattan거리 저장
	for (i = 0; i < MAX_BOARD; i++) {
		int x = (board_p->array[i]) - 1;
		if (x >= 0) {
			index_y = (x / 5);//목표위치의 row값
			index_x = (x % 5);//목표위치의 column값
			score = score 
				+ abs(index_y - (i / 5)) 
				+ abs(index_x - (i % 5));  
		}
	}
	return score;
}

/*         linear conflict        */
int linear(board_t *board_p) {
	int index_x[ROW][COLUMN];
	int index_y[ROW][COLUMN];
	int i,j,k,index = 0;
	int lin_score = 0, man_score;
	
	man_score = manhattan(board_p);                              
	for (i = 0; i<ROW; i++) {
		for (j = 0; j < COLUMN; j++) {
			if (board_p->array[index] == 0) {            //빈칸일때 count 제외
				index_x[i][j] = 100; index++; continue;
			}
			index_x[i][j] = (board_p->array[index++] - 1) / ROW; 	//현재 노드의 목표노드 row값 저장
		}
	}
	index = 0;

	for (i = 0; i<ROW; i++) {
		for (j = 0; j < COLUMN; j++) {
			if (board_p->array[index] == 0) {              //빈칸일때 count 제외 
				index_y[i][j] = 100; index++;  continue;
				//공백은 count에서 제외
			}
			index_y[i][j] = (board_p->array[index++] - 1) % ROW;	//현재 노드의 목표노드 column값 저장
		}
	}
	for (i = 0; i<ROW; i++) {
		for (j = 0; j < COLUMN - 1; j++) {
			for (k = j + 1; k < COLUMN; k++) {
				if (index_x[i][j] == index_x[i][k]) {            //목표노드가 같은 행에 존재할때
					if ((index_y[i][j] != 100) && (index_y[i][j] > index_y[i][k]))   //빈칸이 아니고, 충돌해야 할 경우 count++
						lin_score++;
				}
				if (index_y[k][i] == index_y[j][i]) {            //목표노드가 같은 열에 존재할때
					if ((index_x[j][i] != 100) && (index_x[j][i] > index_x[k][i]))   //빈칸이 아니고, 충돌해야 할 경우 count++
						lin_score++;
				}
			}
		}
	}
	return man_score + 2 * lin_score;
}
/*		inversion의 개수를 세서 풀 수 있는 퍼즐인지 확인		*/
int countInversions(char *array) {
	int i, j, inversions = 0;

	for (j = 0; j < MAX_BOARD; j++) {
			for (i = j + 1; i < MAX_BOARD; i++) {
				if ((array[j]==0) || (array[i]==0)) continue;
				if (array[j] > array[i]) inversions++;
			}
		}
	return inversions;
}

/*         초기상태생성         */
board_t *initPuzzle(void) {
	int i, inversions;
	board_t *board_p;
	board_p = nodeAlloc(); 

	char temp[MAX_BOARD] = { 
		22, 12, 2, 17, 20,
		23, 4, 3, 9, 16,
		10, 15, 24, 1, 13,
		6, 7, 19, 14, 21,
		11, 18, 8, 5, 0 };
	for (i = 0; i < MAX_BOARD; i++) 
		board_p->array[i] = temp[i];
	
	/* 	blank 공간 추적	 */
	for (i = 0; i < MAX_BOARD; i++) {               
		if (board_p->array[i] == 0) {
			board_p->blank = i;
			break;
		}
	}
	board_p->f = board_p->h = linear(board_p);
	board_p->depth = 0;                           
	//초기 노드는 트리의 첫번째 노드가 되므로 깊이가 0

	return board_p;
}

/*1차원배열로 저장한 퍼즐을 행렬로 출력해준다 */
void dispPuzzleBoard(board_t *board) {
	int i;

	assert(board);

	for (i = 0; i < MAX_BOARD; i++) {
		if ((i % 5) == 0) printf("\n");
		if (board->array[i] == 0) printf("   ");
		else printf(" %02d", board->array[i]);
	}
	printf("\n");

	return;
}

#define MAX_VECTOR   4

typedef struct {
	int len;//이동가능한 자리의 수
	unsigned int vector[MAX_VECTOR];//이동할수 있는 index
} move_t;

const move_t moves[MAX_BOARD] = {
	/* 0 */{ 2,{ 1, 5 } },
	/* 1 */{ 3,{ 0, 2, 6 } },
	/* 2 */{ 3,{ 1, 3, 7 } },
	/* 3 */{ 3,{ 2, 4, 8 } },
	/* 4 */{ 2,{ 3, 9 } },
	/* 5 */{ 3,{ 0, 6, 10 } },
	/* 6 */{ 4,{ 1, 5, 7, 11 } },
	/* 7 */{ 4,{ 2, 6, 8, 12 } },
	/* 8 */{ 4,{ 3, 7, 9, 13 } },
	/* 9 */{ 3,{ 4, 8, 14 } },
	/* 10 */{ 3,{ 5, 11, 15 } },
	/* 11 */{ 4,{ 6, 10, 12, 16 } },
	/* 12 */{ 4,{ 7, 11, 13, 17 } },
	/* 13 */{ 4,{ 8, 12, 14, 18 } },
	/* 14 */{ 3,{ 9, 13, 19 } },
	/* 15 */{ 3,{ 10, 16, 20 } },
	/* 16 */{ 4,{ 11, 15, 17, 21 } },
	/* 17 */{ 4,{ 12, 16, 18, 22 } },
	/* 18 */{ 4,{ 13, 17, 19, 23 } },
	/* 19 */{ 3,{ 14, 18, 24 } },
	/* 20 */{ 2,{ 15, 21 } },
	/* 21 */{ 3,{ 16, 20, 22 } },
	/* 22 */{ 3,{ 17, 21, 23 } },
	/* 23 */{ 3,{ 18, 22, 24 } },
	/* 24 */{ 2,{ 19, 23 } } 
};

board_t *getChildBoard(board_t *board_p, int index){
	board_t *child_p = (board_t *)0;
	int     blankSpot;
	int     i;

	blankSpot = board_p->blank;
	/*move.vector의 원소가 최대4개이므로 
	index를 4번반복하며 vector의 index조정을 위해 사용*/
	if (index < moves[blankSpot].len) {
		int moveFrom;

		child_p = nodeAlloc();

		/* board_p의 array를 child_p로 복사 */
		for (i = 0; i < MAX_BOARD; i++)
			child_p->array[i] = board_p->array[i];
		
		child_p->blank = board_p->blank;

		moveFrom = moves[blankSpot].vector[index];         
		//움직일 index를 moveFrom에 저장한다

		/*      공백의 이동      */
		child_p->array[(int)child_p->blank] = child_p->array[moveFrom];
		child_p->array[moveFrom] = 0;
		child_p->blank = moveFrom;
	}
	return child_p;
}

/*   저장해둔 트리의 경로를 따라가면서 solution을 보여준다   */
void showSolution(board_t *goal){
	board_t *revList[MAX_LIST_ELEMENTS];
	int i = 0, j;

	printf("\nSolution:");

	while (goal) {
		revList[i++] = goal;
		goal = goal->pred;
	}

	for (j = i - 1; j >= 0; j--) {
		dispPuzzleBoard(revList[j]);
	}

	printf("\nLength of the solution = %d\n", i - 1);
	printf("Size of open list = %d\n", openList_p.numElements);
	printf("Size of closed list = %d\n", closedList_p.numElements);
	return;
}
/*   A-Star algorithm   */
void astar(void) {
	board_t *cur_board_p, *child_p, *temp;
	int i;

	/*openlist에 노드가 있으면 while문 계속 반복*/
	while (listCount(&openList_p)) {             

		cur_board_p = getListBest(&openList_p);         
		//openlist에서 f score가 작은 element값을 반환

		putList(&closedList_p, cur_board_p);         
		//이미 지나간 element를 closedlist에 넣는다

		if (cur_board_p->h == 0) {         
			//h에 저장된 휴리스틱 값이 0일 경우 퍼즐 완성

			showSolution(cur_board_p);
			return;

		}
		else {

			if (cur_board_p->depth > MAX_DEPTH) continue;            
			//제한된 깊이보다 더 깊어지면 continue해서 다른 element로 탐색한다

			for (i = 0; i < 4; i++) {
				//이동할수 있는 곳으로 4번의 루프동안 한번씩 이동
				child_p = getChildBoard(cur_board_p, i);            

				if (child_p != (board_t *)0) {                     
					//child_p가 가능한 노드로 다 간 경우 null을 반환한다
					int pos;

					if (onList(&closedList_p, child_p->array, NULL)) {  
						//child_p가 closedList에 있는지 확인
						//이미 존재하는 노드일경우 child_p 동적할당 해제
						nodeFree(child_p);                        
						continue;
					}

					child_p->depth = cur_board_p->depth + 1;
					child_p->h = linear(child_p); //linear를 방식에 맞는 휴리스틱 함수명으로 변경
					child_p->g = child_p->depth;
					child_p->f = (child_p->g * 1) + (child_p->h * 2); //h 값에 가중치를 두기 위해 2를 곱한다 

					if (onList(&openList_p, child_p->array, &pos)) { 
						//openlist에 chlid가 있으면 pos를 알아낸다

						temp = delList(&openList_p, pos);         
						//openlist에 pos로 같은 노드를 temp로 반환받는다

						if (temp->g < child_p->g) {               
							//트리에서 똑같은 노드중 깊이가 더 ㅤ얕은걸 선택한다
							nodeFree(child_p);
							putList(&openList_p, temp);
							continue;
						}
						nodeFree(temp);
					}
					child_p->pred = cur_board_p;               
					//tree구조 생성
					putList(&openList_p, child_p);               
					//openlist에 child_p저장
				}
			}
		}
	}
	return;
}

int main(){
	board_t *initial_p;

	initList(&openList_p);
	initList(&closedList_p);

	initial_p = initPuzzle();
	printf("\nInitial:");
	dispPuzzleBoard(initial_p);

	if(countInversions(initial_p->array)&1){
		printf("풀 수 없는 퍼즐입니다.\n"); 
		return 0;
	}
	else
		printf("풀 수 있는 퍼즐입니다.\n");
	
	printf("\nA* linear로 Searching ...\n");  //linear를 방식에 맞는 휴리스틱 함수명으로 변경

	putList(&openList_p, initial_p);
	time_t start, end;
	start = clock();
	astar();
	end = clock();
	printf("\n\n< Initial Puzzle >\n");
	dispPuzzleBoard(initial_p);
	printf("\nA* linear로 Searching 완료\n");  //linear를 방식에 맞는 휴리스틱 함수명으로 변경
	printf("측정시간 : %lf \n", (float)(end - start) / (CLOCKS_PER_SEC));
	cleanupList(&openList_p);
	cleanupList(&closedList_p);

	return 0;
}
