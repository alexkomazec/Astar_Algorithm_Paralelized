// A C++ Program to implement A* Search Algorithm
#include<bits/stdc++.h>
#include <omp.h>
#include <stdio.h>
#include <semaphore.h>
#include <mutex> 
using namespace std;
// To shut the terminal up with "unused varibels". Unused variables are normal
//if there are some selective defines//
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
//---------------------------DEFINES----------------------------------------------------------------//
//-------------NUMBER OF THREADS & DELAY(LOAD)----------------------------//
//#define DEBUG_NUM_OF_TD 1
#define LOAD 0
//------------------------------------------------------------------------//
//-----------ENABLE/DISABLE FUNTION FAILURE PRINT-------------------------//
#define PRINT_ISVALID_FAILURE
#define PRINT_ISUNBLOCKED_FAILURE
#define PRINT_ISDESTINATION_FAILURE
//------------------------------------------------------------------------//
//-----------ENABLE/DISABLE FOUND PATH PRINT------------------------------//
//#define PRINT_FOUND_PATH
//------------------------------------------------------------------------//
//-----------ENABLE/DISABLE BASIC DEBUG PRINT-----------------------------//
#define DEBUG_MESSAGES_SEVERITY_ALL
//-----------ENABLE/DISABLE TIME PERFORMING PRINT-------------------------//
#define PRINT_THE_WHOLE_PROGRAM_PERFORMING_TIME
//#define PRINT_ASTARTSEARCH_TIME_PERFORMING
//#define PRINT_NORTH_SECTION_PERFORMING_TIME
//#define PRINT_SOUTH_SECTION_PERFORMING_TIME
//#define PRINT_EAST_SECTION_PERFORMING_TIME
//#define PRINT_WEST_SECTION_PERFORMING_TIME

//------------------------------------------------------------------------//
//-----------SET TXT LOCATION FROM CPP OR CMD-----------------------------//
//#define IDE_USING
#define CMD_Linux
//------------------------------------------------------------------------//
//-----------PROBLEM INSTANCE AND WIDTH-----------------------------------//
//#define DEBUG_PRBL_WIDE 4
//#define DEBUG_PATH_MTRX "./test_bench/matrix50x50.txt" //Used with IDE_USING
//------------------------------------------------------------------------//
//-----------New Feature Open/Close-----------------------------------//
/*This part of code checks if the side cell is the openList because
  of corner case where thread N inserts the cell in the openList and
  thread N+1 does the same
  NOTE: This part of code should fix the issue but it just make code slower and decrease the speedup of 
  thread 5 and 6*/
//#define OPEN_LIST_DOUBLE_CHECK
//--------------------------------------------------------------------------------------------------//
//---------------------------GLOBAL VARIABLES-------------------------------------------------------//
// Value of column/row
int col_row = 0;
// Creating a shortcut for int, int pair type 
typedef pair<int, int> Pair;
// Creating a shortcut for pair<int, pair<int, int>> type 
typedef pair<double, pair<int, int>> pPair;
//-------------------------------------------------------------------------------------------------//
//--------------------------STRUCTURES-------------------------------------------------------------//
// A structure to hold the neccesary parameters 
struct cell
{
	// Row and Column index of its parent 
	int parent_i;
	int parent_j;
	double f;
	int InOpenList;// This should be a fix related to a problem related to OPEN_LIST_DOUBLE_CHECK
};

//------------------------------------------------------------------------------------------------//
// A Utility Function to check whether given cell (row, col) 
// is a valid cell or not. 
bool isValid(int row, int col)
{
	//printf("row = %d\n",row);
	//printf("col = %d\n",col);
	// Returns true if row number and column number 
	// is in range
	return (row >= 0) && (row < col_row) &&
		(col >= 0) && (col < col_row);
}
// A Utility Function to check whether the given cell is 
// blocked or not 
bool isUnBlocked(vector<vector<int>> grid, int row, int col)
{
	//
	//
	// Returns true if the cell is not blocked else false 
	if (grid[row][col] == 0)
		return (true);
	else
		return (false);
}
// A Utility Function to check whether destination cell has 
// been reached or not 
bool isDestination(int col, int row, Pair dest)
{
	if (row == dest.first && col == dest.second) {
		return (true);
	}
	else
	{
		return (false);
	}
}
// A Utility Function to trace the path from the source 
// to destination 
void tracePath(vector<vector<cell>> cellDetails, vector<vector<char>> output_file_map, Pair dest, Pair src)
{
	int col = dest.first;
	int row = dest.second;
	stack<Pair> Path;
	while (!(cellDetails[row][col].parent_i == row && cellDetails[row][col].parent_j == col))
	{
		output_file_map[row][col] = 'x';
		Path.push(make_pair(row, col));
		int temp_row = cellDetails[row][col].parent_i;
		int temp_col = cellDetails[row][col].parent_j;
		row = temp_row;
		col = temp_col;
		if (src.second == row && src.first == col)
			break;
	}
	
	output_file_map[dest.second][dest.first] = 'E';
	Path.push(make_pair(row, col));
	while (!Path.empty())
	{
		pair<int, int> p = Path.top();
		Path.pop();
		#ifdef PRINT_FOUND_PATH
			printf("-> (%d,%d) ", p.first, p.second);
		#endif // PRINT_FOUND_PATH
	}
	#ifdef PRINT_FOUND_PATH
		printf("\n");
	#endif // PRINT_FOUND_PATH
	std::ofstream outfile;
	outfile.open("res.txt", std::ios_base::out);
	for (int i = 0; i < col_row; i++)
	{
		for (int j = 0; j < col_row; j++)
		{
			outfile << output_file_map[i][j];
		}
		outfile << endl;
	}
	
	return;
}
// A Function to find the shortest path between 
// a given source cell to a destination cell according 
// to A* Search Algorithm 
void aStarSearch(vector<vector<int>> grid, Pair src, Pair dest,int threads)
{   
    sem_t sem;
    std::mutex DestFound;
	std::mutex openListAccess;
	std::mutex payloadTransition; // payload = parent_i,parent_j,f
	double startaStarSearch;
	double endaStarSearch;
	startaStarSearch = omp_get_wtime();
    
	// If the source is out of range 
	if (isValid(src.second, src.first) == false)
	{
		#ifdef PRINT_ISVALID_FAILURE
			printf("Source is invalid\n");
		#endif // PRINT_ISVALID_FAILURE
		return;
	}
	// If the destination is out of range 
	if (isValid(dest.second, dest.first) == false)
	{
		#ifdef PRINT_ISVALID_FAILURE
			printf("Destination is invalid\n");
		#endif // PRINT_ISVALID_FAILURE
		return;
	}
	
	// Either the source or the destination is blocked 
	if (isUnBlocked(grid, src.second, src.first) == false || isUnBlocked(grid, dest.second, dest.first) == false)
	{
		#ifdef PRINT_ISUNBLOCKED_FAILURE
			printf("Source or the destination is blocked\n");
		#endif // PRINT_ISUNBLOCKED_FAILURE
		return;
	}
	// If the destination cell is the same as source cell 
	if (isDestination(src.first, src.second, dest) == true)
	{
		#ifdef PRINT_ISDESTINATION_FAILURE
			printf("We are already at the destination\n");
		#endif // PRINT_ISDESTINATION_FAILURE
		return;
	}
	// output_file_map vector saves parsed matrix that represent the originial input
	vector<vector<char>> output_file_map(col_row, vector<char>(col_row, ' '));
	for (int i = 0; i < col_row; i++)
	{
		for (int j = 0; j < col_row; j++)
		{
			if (i == src.second && j == src.first)
				output_file_map[i][j] = 'S';
			if (i == dest.second && j == dest.first)
				output_file_map[i][j] = 'E';
			if (grid[i][j] == 1)
				output_file_map[i][j] = '1';
		}
	}
    
	/*
	SHORT DESCRIPTION (PURPOSE):
		closeList saves cells/fields that are visited during the searching.
		If the cell/field is in closedList it means the algorithm can't
		visit this cell again.
	FULL DESCRIPTION (STRUCTURE):
		The size of the matrix = col_row x col_row
		//[AK]: 04/12/2020: [DEBUG_PRBL_WIDE][DEBUG_PRBL_WIDE] is only for visual studio version of code!!
							Linux code: bool closedList[col_row][col_row];
	*/
	bool closedList[col_row][col_row];
	memset(closedList, false, sizeof(closedList));
    
	/*
	SHORT DESCRIPTION (PURPOSE):
		Declare a 2D array of structure to hold the details
		of that cell
	FULL DESCRIPTION (STRUCTURE):
		cellDetails vector contains:
		1. parent_i = row of the parent cell
		2. parent_j = col of the parent cell
		3. f = weigth function
	*/
	
	vector<vector<cell> > cellDetails;//(col_row*col_row, vector<cell>(col_row*col_row));
	
	int i, j;
	for ( i = 0; i < col_row; i++) { 
		// Vector to store column elements 
		vector<cell> column; 
        
		for ( j = 0; j < col_row; j++) { 
			column.push_back({-1,-1,FLT_MAX}); 
		} 
		// Pushing back above 1D vector 
		// to create the 2D vector 
		cellDetails.push_back(column); 
	} 
	
	// Initialising the parameters of the starting node 
	i = src.second, j = src.first;
	cellDetails[i][j].f = 0.0;
	cellDetails[i][j].parent_i = i;
	cellDetails[i][j].parent_j = j;
	/*
	SHORT DESCRIPTION (PURPOSE):
		openList saves Temporary Cells (TCs). Capacity of TCS is 4.
		4 = (North, South, East, West).
		Temporary cells will be visited in the future.
	FULL DESCRIPTION (STRUCTURE):
		Create an open list having information as-
		<f, <i, j>>
		f = g,
		<i,j> = <row, col>
		Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
		This open list is implenented as a set of pair of pair.*/
	set<pPair> openList;
	// Put the starting cell on the open list and set its 
	// 'f' as 0 
	openList.insert(make_pair(0.0, make_pair(i, j)));
	//Semaphore
	//sem_t *sem - Specifies the semaphore to be initialized
	//0          - Means it is shared between threads
	//
    sem_init(&sem, 0, 1);
	
	// We set this boolean value as false as initially 
	// the destination is not reached. 
	bool foundDest = false;
	int tid;
	int nthreads;
    omp_set_num_threads(threads);
	//Hardcoded thread arrangmend
	switch(threads) {
      case 1 :
	  	 //Single core
	  	 printf("Single core\n");;
         break;
      case 2 :
	  	 //Two cores
	  	 printf("Two Cores\n");
		 break;
      case 3 :
	  	 //Three cores
	  	 printf("Three cores\n");
         break;
      case 4 :
	  	 //Four cores
	  	 printf("Four cores\n");
         break;
      case 5 :
	  	 //Five cores
	  	 printf("Five cores\n"); 
         break;
	  case 6 :
	  	 //Six cores
	  	 printf("Six cores\n"); 
         break;
      default :
         cout << "Invalid number of cores" << endl;
    }
    pPair p;  
    int firstTimeCheck = 0;
	// To store 'f' of the 4 succesors
	double fNew;
	#pragma omp parallel default(none) private(p,i,j,firstTimeCheck,fNew,tid) shared(DestFound,payloadTransition,openListAccess,sem,col_row,openList,nthreads,foundDest,cellDetails,dest,src,output_file_map,closedList,grid)
	{ /*-2*/
	    firstTimeCheck = 0;
		tid = omp_get_thread_num();
	    sem_wait(&sem);
        
		while (!openList.empty() )
		{ /*-1*/	
			
			if(firstTimeCheck >0)
			{
			    sem_wait(&sem);    
			}
			    
			firstTimeCheck++; //Provides that firstTimeCheck is more than 2.
			
			// p gets the first elemnt of openList
			// Note that it is the first value in previous iteration that is 
			// saved in openList		
		    openListAccess.lock();
			
		if(openList.empty() == true)
		{
		    openListAccess.unlock();
		}
		else
		{//else openList.empty() == false
		    p = *openList.begin();
			openList.erase(openList.begin());
			openListAccess.unlock();
			i = p.second.first;
			j = p.second.second;
			closedList[i][j] = true;
		     
			/*
				Generating all the 4 successor of this cell
						N
						|
					W--Cell--E
						|
						S
				Cell-->Popped Cell (i, j)
				N --> North	 (i-1, j)
				S --> South	 (i+1, j)
				E --> East	 (i, j+1)
				W --> West   (i, j-1) */
  
			nthreads = omp_get_num_threads();
			tid = omp_get_thread_num();
			
			//***   North   ***//
			if(foundDest == false)
			{ /*0*/
		        tid = omp_get_thread_num();	
				//----------- 1st Successor (North) ------------ 
				//i = ROW
				//j = COL
				// Only process this cell if this is a valid one 
				if (isValid(i - 1, j) == true)
				{/*1*/
					// If the destination cell is the same as the 
					// current successor 
					if (isDestination(i - 1, j, dest) == true)
					{/*2*/
						DestFound.lock();
						
						if(foundDest == false)
						{
	                        closedList[i - 1][j] = true;
						
						    //It fixes the problem where one of threads died waiting on semaphore releasing (VALUE = 0)
						    //6 sem_post because there are 6 threads max
						    sem_post(&sem);
						    sem_post(&sem);
						    sem_post(&sem);
						    sem_post(&sem);
						    sem_post(&sem);
						    sem_post(&sem);
						    
						    // Set the Parent of the destination cell 
						    cellDetails[i - 1][j].parent_i = i;
						    cellDetails[i - 1][j].parent_j = j;
							
						    tracePath(cellDetails, output_file_map, dest, src);
						    foundDest = true;
						}
						DestFound.unlock();
					}/*end 2*/
					
					// If the successor is already on the closed 
					// list or if it is blocked, then ignore it. 
					// Else do the following 
					else if (closedList[i - 1][j] == false && isUnBlocked(grid, i - 1, j) == true)
					{/*3*/
					    
						payloadTransition.lock();
						#pragma omp critical
						{
							fNew = cellDetails[i][j].f + 1.0;
						}
						
						// If it isn’t on the open list, add it to 
						// the open list. Make the current square 
						// the parent of this square. Record the 
						// f costs of the square cell 
						//			 OR 
						// If it is on the open list already, check 
						// to see if this path to that square is better, 
						// using 'f' cost as the measure. 
						if (cellDetails[i - 1][j].f == FLT_MAX || cellDetails[i - 1][j].f > fNew)
						{/*4*/
							
							openListAccess.lock();
							if(cellDetails[i - 1][j].InOpenList == 0)
							{ //cellDetails[x][x].InOpenList == 0
								#pragma omp critical
								{
									openList.insert(make_pair(fNew,make_pair(i - 1, j)));
									cellDetails[i - 1][j].InOpenList = 1;
								}
								sem_post(&sem);
								// Update the details of this cell
								//payloadTransition.lock();
								#pragma omp critical
								{ 
									cellDetails[i - 1][j].f = fNew;
									cellDetails[i - 1][j].parent_i = i;
									cellDetails[i - 1][j].parent_j = j;
								}
							} //cellDetails[x][x].InOpenList == 0
														
							openListAccess.unlock();
						} /*4 end*/ //end of if (cellDetails[i - 1][j].f == FLT_MAX || cellDetails[i - 1][j].f > fNew
					     payloadTransition.unlock();
					}/*3 end*/ //end of else if (closedList[i - 1][j] == false && isUnBlocked(grid, i - 1, j) == true)
				}/*1 end*/ // end of isValid
			}/*0 end*/ //end of if (foundDest == false)
			
			//***   South   ***//
			if (foundDest == false)
			{ /*0*/
				tid = omp_get_thread_num();
				
				//----------- 2nd Successor (South) ------------ 
				// Only process this cell if this is a valid one 
				
				if (isValid(i + 1, j) == true)
				{/*1*/
					// If the destination cell is the same as the 
					// current successor 
					
					if (isDestination(i + 1, j, dest) == true)
					{/*2*/
						DestFound.lock();
						if(foundDest == false)
						{
						    closedList[i + 1][j] = true;
						    
						    //It fixes the problem where one of threads died waiting on semaphore releasing (VALUE = 0)
						    //6 sem_post because there are 6 threads max
						    sem_post(&sem);
						    sem_post(&sem);
						    sem_post(&sem);
						    sem_post(&sem);
						    sem_post(&sem);
						    sem_post(&sem);
						    
						    // Set the Parent of the destination cell 
					        cellDetails[i + 1][j].parent_i = i;
					        cellDetails[i + 1][j].parent_j = j;

						    tracePath(cellDetails, output_file_map, dest, src);
						    
						    foundDest = true;
						}						
						DestFound.unlock();
					}/*end 2*/
					// If the successor is already on the closed 
					// list or if it is blocked, then ignore it. 
					// Else do the following 
					else if (closedList[i + 1][j] == false && isUnBlocked(grid, i + 1, j) == true)
					{/*3*/
						payloadTransition.lock();
						#pragma omp critical
						{
						    fNew = cellDetails[i][j].f + 1.0;
						}
						
						// If it isn’t on the open list, add it to 
						// the open list. Make the current square 
						// the parent of this square. Record the 
						// f costs of the square cell 
						//			 OR 
						// If it is on the open list already, check 
						// to see if this path to that square is better, 
						// using 'f' cost as the measure. 
						if (cellDetails[i + 1][j].f == FLT_MAX || cellDetails[i + 1][j].f > fNew)
						{/*4*/
							openListAccess.lock();
														
							if(cellDetails[i + 1][j].InOpenList == 0)
							{ //cellDetails[x][x].InOpenList == 0
							
								#pragma omp critical
								{
									openList.insert(make_pair(fNew, make_pair(i + 1, j)));
									cellDetails[i + 1][j].InOpenList = 1;
								}
								sem_post(&sem);
								// Update the details of this cell
								//payloadTransition.lock(); 
								#pragma omp critical
								{	    
									cellDetails[i + 1][j].f = fNew;
									cellDetails[i + 1][j].parent_i = i;
									cellDetails[i + 1][j].parent_j = j;
								}
							} //cellDetails[x][x].InOpenList == 0
	                            									
							openListAccess.unlock();   
							} /*4 end*/
							payloadTransition.unlock();	
						} /*3 end*/
					}/*1 end*/
				} /*0 end*/
			
			//***   EAST   ***//	
			//Mesure East section time
			if (foundDest == false)
			{/*0*/
				tid = omp_get_thread_num();				
			
				//----------- 3rd Successor (East) ------------ 
				
				// Only process this cell if this is a valid one 
				if (isValid(i, j + 1) == true)
				{/*1*/
					// If the destination cell is the same as the 
					// current successor 
					
					if (isDestination(i, j + 1, dest) == true)
					{/*2*/
						DestFound.lock();
						
						if(foundDest == false)
						{
						    closedList[i][j+1] = true;
						
						    //It fixes the problem where one of threads died waiting on semaphore releasing (VALUE = 0)
						    //6 sem_post because there are 6 threads max
						    sem_post(&sem);
						    sem_post(&sem);
						    sem_post(&sem);
						    sem_post(&sem);
						    sem_post(&sem);
						    sem_post(&sem);
						    
						    //Set the Parent of the destination cell
						    cellDetails[i][j + 1].parent_i = i;
						    cellDetails[i][j + 1].parent_j = j;
						
						    tracePath(cellDetails, output_file_map, dest, src);
						    foundDest = true;
						}
						
						DestFound.unlock();
					}/*end 2*/
					
					// If the successor is already on the closed 
					// list or if it is blocked, then ignore it. 
					// Else do the following 
					else if (closedList[i][j + 1] == false && isUnBlocked(grid, i, j + 1) == true)
					{/*3*/
					    
						payloadTransition.lock();
					    #pragma omp critical
						{
						    fNew = cellDetails[i][j].f + 1.0;
						}
						
						// If it isn’t on the open list, add it to 
						// the open list. Make the current square 
						// the parent of this square. Record the 
						// f cost of the square cell 
						//			 OR 
						// If it is on the open list already, check 
						// to see if this path to that square is better, 
						// using 'f' cost as the measure. 
						if (cellDetails[i][j + 1].f == FLT_MAX || cellDetails[i][j + 1].f > fNew)
						{/*4*/
							openListAccess.lock();
							
							if(cellDetails[i][j + 1].InOpenList == 0)
							{ //cellDetails[x][x].InOpenList == 0
							
								#pragma omp critical
								{
									openList.insert(make_pair(fNew,make_pair(i, j + 1)));
									cellDetails[i][j + 1].InOpenList = 1;
								}
								sem_post(&sem);

								// Update the details of this cell
								#pragma omp critical
								{ 
									cellDetails[i][j + 1].f = fNew;
									cellDetails[i][j + 1].parent_i = i;
									cellDetails[i][j + 1].parent_j = j;
								}
							} //cellDetails[x][x].InOpenList == 0
							openListAccess.unlock();
						}/*4 end*/
						payloadTransition.unlock();	
					}/*3 end*/
				}/*1 end*/
			}/*0 end*/
					
			//***   West   ***//	
			if (foundDest == false)
			{/*0*/
				tid = omp_get_thread_num();
					
				//----------- 4th Successor (West) ------------ 
				// Only process this cell if this is a valid one 
				if (isValid(i, j - 1) == true)
				{/*1*/
					// If the destination cell is the same as the 
					// current successor
					
					if (isDestination(i, j - 1, dest) == true)
					{/*2*/
						// Set the Parent of the destination cell
						DestFound.lock();
						
						if(foundDest == false)
						{
						    closedList[i][j - 1] = true;
						    
					        //It fixes the problem where one of threads died waiting on semaphore releasing (VALUE = 0)
						    //6 sem_post because there are 6 threads max
						    sem_post(&sem);
						    sem_post(&sem);
						    sem_post(&sem);
						    sem_post(&sem);
						    sem_post(&sem);
						    sem_post(&sem);
						    
						    cellDetails[i][j - 1].parent_i = i;
						    cellDetails[i][j - 1].parent_j = j;
						    tracePath(cellDetails, output_file_map, dest, src);
							foundDest = true;
						}
						DestFound.unlock();
					}/*2 end*/
						// If the successor is already on the closed 
						// list or if it is blocked, then ignore it. 
						// Else do the following 
						else if (closedList[i][j - 1] == false && isUnBlocked(grid, i, j - 1) == true)
						{/*3*/
							payloadTransition.lock();
						    #pragma omp critical
						    {
							    fNew = cellDetails[i][j].f + 1.0;
							}
							//payloadTransition.unlock();
							// If it isn’t on the open list, add it to 
							// the open list. Make the current square 
							// the parent of this square. Record the 
							// f cost of the square cell 
							//			 OR 
							// If it is on the open list already, check 
							// to see if this path to that square is better, 
							// using 'f' cost as the measure. 
							if (cellDetails[i][j - 1].f == FLT_MAX || cellDetails[i][j - 1].f > fNew)
							{/*4*/
								openListAccess.lock();
								
								if(cellDetails[i][j - 1].InOpenList == 0)
							    { //cellDetails[x][x].InOpenList == 0
								
									#pragma omp critical
									{
										openList.insert(make_pair(fNew,make_pair(i, j - 1)));
										cellDetails[i][j - 1].InOpenList = 1;
										
									}
								    sem_post(&sem);
									
									// Update the details of this cell
									#pragma omp critical
									{
										cellDetails[i][j - 1].f = fNew;
										cellDetails[i][j - 1].parent_i = i;
										cellDetails[i][j - 1].parent_j = j;
									}
							    }   //cellDetails[x][x].InOpenList == 0							
								openListAccess.unlock();
							}/*4 end*/ // if (cellDetails[i][j - 1].f == FLT_MAX || cellDetails[i][j - 1].f > fNew)
							payloadTransition.unlock();
						}/*3 end*/ // else if (closedList[i][j - 1] == false && isUnBlocked(grid, i, j - 1) == true)
				}/*1*/ // if (isValid(i, j - 1) == true)
			} /*0 end */ 
		}//else openList.empty() == false
	}/*-1*/ //End of while
}/*-2 end*/ // pragma omp block of code

	// When the destination cell is not found and the open 
	// list is empty, then we conclude that we failed to 
	// reach the destiantion cell. This may happen when the 
	// there is no way to destination cell (due to blockages) 
	if (foundDest == false) {
		#ifdef DEBUG_MESSAGES_SEVERITY_ALL
			printf("Failed to find the Destination Cell\n");
		#endif // DEBUG_MESSAGES_SEVERITY_ALL
	}
	else{
        
	endaStarSearch = omp_get_wtime();
		
	#ifdef PRINT_ASTARTSEARCH_TIME_PERFORMING
		printf("*********************************************************************************\n");
		printf("~Work took aStarSearch Section %f seconds\n~", (endaStarSearch - startaStarSearch));
		printf("*********************************************************************************\n");
	#endif // PRINT_ASTARTSEARCH_TIME_PERFORMING
	
    return;
	}
		
}
int MatrixInfo_txt(string path_name, string& map)
{
	ifstream input(path_name);
	for (int i = 0; !input.eof(); i++)
	{
		map.resize(i + 1);
		input >> map[i];
	}
	return map.size() - 1;
}
void getInputSize(int *col_row,int matrix_size)
{
    int done = 0;
    
     for(int i= 1;!done;i++)
     {
         *col_row = matrix_size/i;
         if(*col_row ==i){
            done =1;
         }
     }
}
// Driver program to test above function 
int main(int argc, char* argv[])
{
    
	double start;
	double end;
	start = omp_get_wtime();
	
#ifdef CMD_Linux
	if (argc == 3) {
	
	}
	else if (argc > 3) {
		printf("Too many arguments supplied.\n");
		printf("Please, call binary like\n");
		printf("./Astar example.txt 4\n");
		exit(1);
	}
	else {
		printf("Too few arguments supplied.\n");
		printf("Please, call binary like\n");
		printf("./Astar example.txt\n");
		exit(2);
	}
#endif // CMD_Linux
#ifdef CMD_Linux
	string path_name = argv[1];
	int threads = (int)(*argv[2]) - 48;
#endif //CMD_Linux
#ifdef IDE_USING
	string path_name = DEBUG_PATH_MTRX;
#endif
	string map = "";
	int matrix_size = MatrixInfo_txt(path_name, map);
    
	getInputSize(&col_row,matrix_size);
	
	int  src_dest_row[2]; // [1] source row [2] destination row
	int  src_dest_col[2]; // [1] source col [2] destination col
	
	vector<vector<int>> grid(col_row, vector<int>(col_row, 0));

	for (int i = 0; i < col_row; i++)
	{
		for (int j = 0; j < col_row; j++)
		{
			if (map[i * col_row + j] == 'S') {
				grid[i][j] = (int)map[i * col_row + j] - 'S';
				src_dest_row[0] = i;
				src_dest_col[0] = j;
			}
			if (map[i * col_row + j] == 'E') {
				grid[i][j] = (int)map[i * col_row + j] - 'E';
				src_dest_row[1] = i;
				src_dest_col[1] = j;
			}
			if (map[i * col_row + j] == '0' || map[i * col_row + j] == '1')
				grid[i][j] = (int)map[i * col_row + j] - '0';
		}
	}

	// Source 
	Pair src = make_pair(src_dest_col[0], src_dest_row[0]);
	// Destination
	Pair dest = make_pair(src_dest_col[1], src_dest_row[1]);
	
	aStarSearch(grid, src, dest,threads);
	
	end = omp_get_wtime();
	#ifdef PRINT_THE_WHOLE_PROGRAM_PERFORMING_TIME
		printf("****************************************************************\n");
		printf("~Work took %f seconds\n~", (end - start));
		printf("****************************************************************\n");
	#endif // PRINT_THE_WHOLE_PROGRAM_PERFORMING_TIME
	
	
	return(0);
}

