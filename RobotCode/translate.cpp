#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <stack>
#include <unistd.h>

using namespace std;

// split the instruction string using commas as delimiter
vector<string> splitInstructions(string webCode){
    vector<string> result;
    stringstream ss (webCode);
    string item;
    while (getline (ss, item, ',')) {
        result.push_back (item);
    }
    return result;
}

// returns the index of the corresponding closing bracket
// takes the index of the opening bracket as a parameter
int indexLastBracket(int start, vector<char> firstLetters) {
    int bracketCloseCount = 1; // tracks closing/opening brackets
    while(bracketCloseCount != 0){
        start++;
        if(firstLetters[start] == '}' || firstLetters[start] == 'g'){
            bracketCloseCount--;
        } else if(firstLetters[start] == 'p' || firstLetters[start] == 'c' || firstLetters[start] == 'i' || firstLetters[start] == 'x'){
            bracketCloseCount++;
        }
    }
    return start;
}

// converts the ending brackets of loops to (goto start of loop)
void endBracketsToLoopGotos(vector<string> *instructions, vector<char> *firstLetters){
    for(int i = 0; i < instructions->size(); i++){
        if((*firstLetters)[i] == 'p' || (*firstLetters)[i] == 'c'){
            int lastBracket = indexLastBracket(i, *firstLetters);
            (*instructions)[lastBracket] = "g" + to_string(i);
            (*firstLetters)[lastBracket] = 'g';
        }
    }
}

int main()
{
    string inputString = "w1,w2,w3,w10";//"m1,p2,f1,p7,f44,},i1,w3,},},k,f2";
    //                    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13
    vector<string> instructions = splitInstructions(inputString);
    vector<char> firstLetters;
    stack<int> loopReps;
    for(int i = 0; i < instructions.size(); i++){
        firstLetters.push_back(instructions[i].at(0));
    }
    endBracketsToLoopGotos(&instructions, &firstLetters);
    
    int index = 0;  // index of current instruction being processed
    
    while(index < instructions.size()){
        switch(firstLetters[index]){
            case 'f':
                cout << "Moving forward " << instructions[index].substr(1) << " feet" << endl;
                break;
            case 'k':
                cout << "Moving forward until path blocked" << endl;
                break;
            case 'b':
                cout << "Moving backward" << instructions[index].substr(1) << " feet" << endl;
                break;
            case 'l':
                cout << "Turning left" << endl;
                break;
            case 'r':
                cout << "Turning right" << endl;
                break;
            case 'm':
                cout << "Moving to <insert location>" << endl;
                break;
            case 'p':{
                int reps = stoi(instructions[index].substr(1)); // original number of repeats
                loopReps.push(reps);
                instructions[index] = "x" + to_string(reps - 1);
                firstLetters[index] = 'x';
                }break;
            case 'x':{
                // repeat x times (during iterations)
                int curReps = stoi(instructions[index].substr(1));
                if(curReps == 0){
                    instructions[index] = "p" + to_string(loopReps.top());
                    firstLetters[index] = 'p';
                    loopReps.pop();
                    index = indexLastBracket(index, firstLetters); // will increment beyond index of last bracket later
                } else {
                    instructions[index] = "x" + to_string(curReps - 1);
                } 
                }break;
            case 'c':   // while loop is just an if statement with "goto start" at end
            case 'i':{
                int condition = stoi(instructions[index].substr(1));
                bool eval = false;
                if(condition == 1){
                    // if path blocked
                    eval = true; // insert completed boolean here
                } else if(condition == 2){
                    eval = true; // insert completed boolean here
                }
                
                if(!eval){
                    index = indexLastBracket(index, firstLetters); // if false, skip past block
                }
                }break;
            case 'w':
                cout << "Waiting " << instructions[index].substr(1) << " seconds" << endl;
                usleep(stoi(instructions[index].substr(1)) * 1000000);
                break;
            case 's':
                cout << "Saying something" << endl;
                break;
            case 'g':
                // goto instruction, jump to specific command
                // not part of official command set, } is converted to this
                index = stoi(instructions[index].substr(1)) - 1; // extract index
                break;
            case '}': break;
            default:
                cout << "There is an error with the input" << endl;
                break;
        }
        index++;
    }
    
    return 0;
}
