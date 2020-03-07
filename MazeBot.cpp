#include <cstdlib>
#include <iostream>
#include <math.h>
#include <deque>
#include <string>
#include <fstream>

using namespace std;

/* Constants */

static const int BOARDSIZE = 25; // 25 x 25 maze read in from map.txt

// Used to define the various blocks on the map
static const int FREE = 1; // Empty space
static const int START = 4; // Starting position
static const int BLOCK = 5; // Represents a maze wall
static const int OPEN = 6; // Node added to open queue but not yet expanded
static const int CLOSED = 7; // After node is expanded and explores other nodes around it 
static const int E1 = 8; // Exit marked E1
static const int E2 = 9; // Exit marked E2
static const int CORNER = 0; // Exit located in top right corner 

// Flags for keeping track of which search technique is currently in use 
static bool BFS = true; // Breadth First Search -> Default starting search method
static bool DFS = false; // Depth First Search
static bool AS = false; // A* search

/* Coord */

class Coord // Holds coordinates and attributes of places in the maze
{
    private:
        int row, col; // Stores position of the agent
        int depth; // Stores the agent's current depth
        int functionF; // f(n) = g(n) + h(n), estimated total path cost 
        Coord* parent; // Keep track of parent address

    public:
        // Constructor used for BFS and DFS
        Coord(int row, int col, int depth, Coord* parent)
        {
            this->row = row;
            this->col = col;
            this->depth = depth;
            this->parent = parent;
        }

        // Constructor used for A* search
        Coord(int row, int col, int depth, int functionF, Coord* parent)
        {
            this->row = row;
            this->col = col;
            this->depth = depth;
            this->functionF = functionF;
            this->parent = parent;
        }

        // Delete the pointer to the parent node 
        ~Coord() { delete parent; }

        // Accessor methods
        int getRow() { return row; }
        int getCol() { return col; }
        int getDepth() { return depth; }
        int getFunctionF() { return functionF; }
        Coord* getParent() { return parent; }
};

/* Search */

class Search
{
    private:
        int mazeMap[BOARDSIZE][BOARDSIZE]; // Two dimensional array for mapping the original maze
        int mazeRoute[BOARDSIZE][BOARDSIZE]; // Two dimensional array for keeping track of open and closed nodes

        int startRow, startCol; // Store the starting position of the agent
        int startE1E2Row, startE1E2Col; // Store the starting position for searching for E1 or E2
        int exitRow, exitCol; // Store exit coordinates, either E1 or E2
        int exitE1Row, exitE1Col; // Store coordinates of E1
        int exitE2Row, exitE2Col; // Store coordinates of E2

        int cost; // Total number of moves made 
        int maxOpenQSize; // Keep track of maximum open queue size to report on memory performance

        // Deques are implemented for their use as a queue and stack
        deque<Coord*> openDeque;
        deque<Coord*> closedDeque;

    public:
        Search();

        void SearchTemplate(int);
        void SearchImplementation(int, int);
        void AStarSort(int, int, int, Coord*);

        void Print();
        void CleanUp();
        void SwitchSearch();
};

/* Search constructor/destructor */

Search::Search() : cost(0), maxOpenQSize(0)
{
    ifstream mapFile;
    mapFile.open("map.txt", ios::in);

    if (mapFile.is_open())
    {
        // Initialize both maps 
        for (int i = 0; i < BOARDSIZE; i++)
        {
            for (int j = 0; j < BOARDSIZE; j++)
            {
                mapFile >> mazeMap[i][j];
                mazeRoute[i][j] = mazeMap[i][j];

                // Mark the beginning and exit coordinates
                if (mazeMap[i][j] == START)
                {
                    startE1E2Row = i;
                    startE1E2Col = j;
                }
                else if (mazeMap[i][j] == E1)
                {
                    exitE1Row = i;
                    exitE1Col = j;
                }
                else if (mazeMap[i][j] == E2)
                {
                    exitE2Row = i;
                    exitE2Col = j;
                }
            }
        }

        mapFile.close();
    }
    else 
    {
        cout << "Unable to open file";
    }
}

/* Search template */

void Search::SearchTemplate(int exit)
{
    // Define starting position and which exit we are searching for
    if (exit == E1)
    {
        startRow = startE1E2Row;
        startCol = startE1E2Col;

        exitRow = exitE1Row;
        exitCol = exitE1Col;
    }
    else if (exit == E2)
    {
        startRow = startE1E2Row;
        startCol = startE1E2Col;

        exitRow = exitE2Row;
        exitCol = exitE2Col;
    }
    else if (exit == CORNER)
    {
        startRow = BOARDSIZE - 1;
        startCol = 0;

        exitRow = 0;
        exitCol = BOARDSIZE - 1;
    }

    // Load the starting coordinates into the open queue
    if (BFS)
    {
        openDeque.push_back(new Coord(startRow, startCol, 0, 0));
    }
    else if (DFS)
    {
        openDeque.push_front(new Coord(startRow, startCol, 0, 0));
    }
    else if (AS)
    {
        int functionF = abs(startRow - exitRow) + abs(startCol - exitCol);
        openDeque.push_back(new Coord(startRow, startCol, 0, functionF, 0));
    }

    maxOpenQSize = 1;

    mazeRoute[startRow][startCol] = OPEN;

    // Keep track of where we are in the maze
    int row = 0;
    int col = 0;

    bool win = false; // Did we solve the maze?

    // Keeping searching as an exit is determined or no solution is found
    while (openDeque.size() != 0)
    {
        // Get the row and column of the current position
        row = openDeque.front()->getRow();
        col = openDeque.front()->getCol();

        mazeRoute[row][col] = CLOSED; // Current position has been opened and explored
        cost++; // Increase cost for each node explored

        if (mazeMap[row][col] == exit) // Check if the goal has been found
        {
            closedDeque.push_back(openDeque.front());
            openDeque.pop_front();

            win = true; // Goal has been found
            break;
        }
        else // No goal found, we must check surrounding nodes
        {
            closedDeque.push_back(openDeque.front());
            openDeque.pop_front(); // Dispose of the first element in the deque

            SearchImplementation(row, col);
        }
    }

    if (!win)
    {
        cout << "This maze has no solution!" << endl;
    }

    Print();
}

/* Breadth First/AStar search */

void Search::SearchImplementation(int row, int col)
{
    // Use this method to push_back nodes for BFS and call insertion sort for A*
    int depth = (closedDeque.back()->getDepth()) + 1; // Depth of the next node will be +1 of the current node recently added to the closed deque
    Coord* parent = closedDeque.back(); // Parent address is the previous node

    // Try left of the current position
    if ((col - 1 > -1) && mazeRoute[row][col - 1] != BLOCK && mazeRoute[row][col - 1] != CLOSED && mazeRoute[row][col - 1] != OPEN)
    {
        mazeRoute[row][col - 1] = OPEN;

        if (BFS)
        {
            openDeque.push_back(new Coord(row, col - 1, depth, parent));
        }
        else if (DFS)
        {
            openDeque.push_front(new Coord(row, col - 1, depth, parent));
        }
        else if (AS)
        {
            AStarSort(row, col - 1, depth, parent);
        }
    }

    // Try above the current position
    if ((row - 1 > -1) && mazeRoute[row - 1][col] != BLOCK && mazeRoute[row - 1][col] != CLOSED && mazeRoute[row - 1][col] != OPEN)
    {
        mazeRoute[row - 1][col] = OPEN;

        if (BFS)
        {
            openDeque.push_back(new Coord(row - 1, col, depth, parent));
        }
        else if (DFS)
        {
            openDeque.push_front(new Coord(row - 1, col, depth, parent));
        }
        else if (AS)
        {
            AStarSort(row - 1, col, depth, parent);
        }
    }

    // Try right of the current position
    if ((col + 1 < BOARDSIZE) && mazeRoute[row][col + 1] != BLOCK && mazeRoute[row][col + 1] != CLOSED && mazeRoute[row][col + 1] != OPEN)
    {
        mazeRoute[row][col + 1] = OPEN;

        if (BFS)
        {
            openDeque.push_back(new Coord(row, col + 1, depth, parent));
        }
        else if (DFS)
        {
            openDeque.push_front(new Coord(row, col + 1, depth, parent));
        }
        else if (AS)
        {
            AStarSort(row, col + 1, depth, parent);
        }
    }

    // Try below the current position
    if ((row + 1 < BOARDSIZE) && mazeRoute[row + 1][col] != BLOCK && mazeRoute[row + 1][col] != CLOSED && mazeRoute[row + 1][col] != OPEN)
    {
        mazeRoute[row + 1][col] = OPEN;

        if (BFS)
        {
            openDeque.push_back(new Coord(row + 1, col, depth, parent));
        }
        else if (DFS)
        {
            openDeque.push_front(new Coord(row + 1, col, depth, parent));
        }
        else if (AS)
        {
            AStarSort(row + 1, col, depth, parent);
        }
    }

    maxOpenQSize = (openDeque.size() > maxOpenQSize) ? openDeque.size() : maxOpenQSize;
}

/* AStar search */

void Search::AStarSort(int row, int col, int depth, Coord* parent)
{
    // A* search uses a priority queue
    // Insert elements into the deque according to priority
    int gN = depth;
    int hN = abs(exitRow - row) + abs(exitCol - col);
    int functionF = gN + hN;

    bool insertSuccess = false;

    // Use insertion sort to insert the element into deque according to highest priority (i.e. lowest functionF value)
    if (openDeque.size() == 0)
    {
        openDeque.push_back(new Coord(row, col, depth, functionF, parent));
    }
    else 
    {
        deque<Coord*>::iterator it;
        int i = 0;

        for (it = openDeque.begin(); it != openDeque.end(); it++)
        {
            if (functionF < openDeque[i]->getFunctionF())
            {
                openDeque.insert(it, new Coord(row, col, depth, functionF, parent));

                insertSuccess = true;
                break;
            }

            i++;
        }

        if (!insertSuccess)
        {
            openDeque.push_back(new Coord(row, col, depth, functionF, parent));
        }
    }
}

/* Maintenance */

void Search::Print()
{
    // Building the path taken by the agent by climbing the tree from the exit state 
    Coord* treeIterator;
    deque<Coord*> path;

    for (treeIterator = closedDeque.back(); treeIterator->getParent() != 0; treeIterator = treeIterator->getParent())
    {
        path.push_front(treeIterator);
    }

    path.push_front(treeIterator);

    // Display the path found in the maze
    // Use tokens that are easy to read
    cout << "\nPath Taken" << endl;

    for (int i = 0; i < BOARDSIZE; i++)
    {
        for (int j = 0; j < BOARDSIZE; j++)
        {
            if (mazeMap[i][j] == BLOCK)
            {
                cout << "¦";
            }
            else if (mazeMap[i][j] == START)
            {
                cout << "S";
            }
            else if (mazeMap[i][j] == E1)
            {
                cout << "E";
            }
            else if (mazeMap[i][j] == E2)
            {
                cout << "F";
            }
            else 
            {
                bool on_the_path = false;

                for (int k = 0; k < path.size(); k++)
                {
                    if (path[k]->getRow() == i && path[k]->getCol() == j)
                    {
                        on_the_path = true;
                    }

                    if (on_the_path)
                    {
                        cout << ".";
                    }
                    else 
                    {
                        cout << " ";
                    }
                }
            }
        }

        cout << "\n";
    }

    cout << "\nComplete path: " << endl;

    for (int k = 0; k < path.size(); k++)
    {
        cout << "(" << (path[k]->getCol()) << "," << BOARDSIZE - (path[k]->getRow()) - 1 << ")";

        if (k < path.size() - 1)
        {
            cout << " -> ";
        }
    }

    cout << endl << endl;

    cout << "Path Cost: " << path.size() << endl;
    cout << "Total Cost: " << cost - 1 << endl; // Don't count the initial state
    cout << "Maximum Size of Open Queue (fringe): " << maxOpenQSize << endl;
    cout << "Final Size of Open Queue: " << openDeque.size() << endl;
    cout << "Final Size of Closed Queue (expanded states): " << closedDeque.size() << endl;
    cout << "Total number of explored states (whether expanded or not): " << openDeque.size() + closedDeque.size() << endl;
}

void Search::CleanUp()
{
    // Reintialize the maze
    for (int i = 0; i < BOARDSIZE; i++)
    {
        for (int j = 0; j < BOARDSIZE; j++)
        {
            mazeRoute[i][j] = mazeMap[i][j];
        }
    }

    // Clear out the deque and reset variables
    openDeque.clear();
    closedDeque.clear();

    cost = 0;
    maxOpenQSize = 0;
}

void Search::SwitchSearch()
{
    // Specify the next search to be completed
    if (BFS)
    {
        BFS = false;
        DFS = true;
    }
    else if (DFS)
    {
        DFS = false;
        AS = true;
    }
}

/* Main */

int main()
{
    Search * s = new Search();

    cout << "BREADTH FIRST SEARCH" << endl;

    cout << "\n---------------------------------------------------------\n" << endl;

    cout << "Search for E1" << endl << endl;
    s->SearchTemplate(E1); // Try to find E1
    s->CleanUp();

    cout << "\n---------------------------------------------------------\n" << endl;

    cout << "Search for E2" << endl << endl;
    s->SearchTemplate(E2); // Try to find E2
    s->CleanUp();

    cout << "\n---------------------------------------------------------\n" << endl;

    cout << "Search for Corner Exit" << endl << endl;
    s->SearchTemplate(CORNER);
    s->CleanUp();
    s->SwitchSearch(); // Switch to DFS

    cout << "\n---------------------------------------------------------\n" << endl;

    cout << "DEPTH FIRST SEARCH" << endl;

    cout << "\n---------------------------------------------------------\n" << endl;

    cout << "Search for E1" << endl << endl;
    s->SearchTemplate(E1); 
    s->CleanUp();

    cout << "\n---------------------------------------------------------\n" << endl;

    cout << "Search for E2" << endl << endl;
    s->SearchTemplate(E2); 
    s->CleanUp();

    cout << "\n---------------------------------------------------------\n" << endl;

    cout << "Search for Corner Exit" << endl << endl;
    s->SearchTemplate(CORNER);
    s->CleanUp();
    s->SwitchSearch(); // Switch to AS

    cout << "\n---------------------------------------------------------\n" << endl;

    cout << "ASTAR SEARCH" << endl;

    cout << "\n---------------------------------------------------------\n" << endl;

     cout << "Search for E1" << endl << endl;
    s->SearchTemplate(E1); 
    s->CleanUp();

    cout << "\n---------------------------------------------------------\n" << endl;

    cout << "Search for E2" << endl << endl;
    s->SearchTemplate(E2); 
    s->CleanUp();

    cout << "\n---------------------------------------------------------\n" << endl;

    cout << "Search for Corner Exit" << endl << endl;
    s->SearchTemplate(CORNER);
    s->CleanUp();

    // Clean up allocated memory
    delete s;

    return EXIT_SUCCESS;
}