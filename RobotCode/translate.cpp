#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <stack>

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

int main()
{
    string inputString = "m1,p2,f1,p7,f44,},i1,w3,l2,},}";
    vector<string> instructions = splitInstructions(inputString);
    vector<char> firstLetters;
    stack<int> loopReps;
    for(int i = 0; i < instructions.size(); i++)
        firstLetters.push_back(instructions[i].at(0));
    
    int index = 0;  // index of current instruction being processed
    
    while(index < instructions.size()){
        switch(firstLetters[index]){
            case 'f':
                cout << "Moving forward " << instructions[index].substr(1) << " feet" << endl;
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
                // repeat # times (starting iterations)
                int lastBracketIndex = indexLastBracket(index, firstLetters);
                instructions[lastBracketIndex] = "g" + to_string(index);
                firstLetters[lastBracketIndex] = 'g';
                
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
                    loopReps.pop();
                    index = indexLastBracket(index, firstLetters); // will increment beyond index of last bracket later
                } else {
                    instructions[index] = "x" + to_string(curReps - 1);
                } 
                }break;
            case 'c':
                break;
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
                    index = indexLastBracket(index, firstLetters);
                }
                }break;
            case 'w':
                cout << "Waiting " << instructions[index].substr(1) << " seconds" << endl;
                break;
            case 's':
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
