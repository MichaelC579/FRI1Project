#include <iostream>
#include <sstream>
#include <string>
#include <vector>

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
        if(firstLetters[start] == "}" || firstLetters[start] == "g"){
            bracketCloseCount--;
        } else if(firstLetters[start] == "p" || firstLetters[start] == "c" || firstLetters[start] == "i"){
            bracketCloseCount++;
        }
    }
    return start;
}

int main()
{
    string inputString = "m1,p3,}";
    vector<string> instructions = splitInstructions(inputString);
    vector<char> firstLetters;
    for(int i = 0; i < instructions.size(); i++)
        firstLetters.push_back(instructions[i].at(0));
    
    int index = 0;  // index of current instruction being processed
    
    while(index < instructions.size()){
        switch(firstLetters[index]){
            case 'f':
                cout << "Moving forward" << endl;
                break;
            case 'b':
                cout << "Moving backward" << endl;
                break;
            case 'l':
                cout << "Turning left" << endl;
                break;
            case 'r':
                cout << "Turning right" << endl;
                break;
            case 'm':
                break;
            case 'p':
                // repeat # times
                int lastBracketIndex = indexLastBracket(index);
                instructions[lastBracketIndex] = "g" + index;
                break;
            case 'c':
                break;
            case 'i':
                break;
            case 'w':
                break;
            case 's':
                break;
            case 'g':
                // goto instruction, jump to specific command
                // not part of official command set, } is converted to this
                index = stoi(instructions[index].substr(1)); // extract index
                break;
            default:
                cout << "There is an error with the input" << endl;
                break;
        }
        index++;
    }
    
    return 0;
}
