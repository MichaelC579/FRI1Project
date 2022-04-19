#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

vector<string> translate(string webCode){
    vector<string> result;
    stringstream ss (webCode);
    string item;

    while (getline (ss, item, ',')) {
        result.push_back (item);
    }

    return result;
}

//will return the index of the last bracket + 1
int indexLastBracket(int start, vector<char> firstLetters) {
    int bracketCloseCount = 1;
    while(bracketCloseCount != 0){
        if(code[curLine] == "}"){
          bracketCloseCount--;
        }
        else if(firstLetters[tempIndex] == "p" || firstLetters[tempIndex] == "c" || firstLetters[tempIndex] == "i"){
          bracketCloseCount++;
        }
        start++;
    }
    return start;
}

int main()
{
    string test = "m1,fw,213,1rfi,12222";
    vector<string> instructions = translate(test);
    vector<char> firstLetters;
    for(int i = 0; i < instructions.size(); i++)
        firstLetters.push_back(instructions[i].at(0));
    
    /*for(int i = 0; i < instructions.size(); i++) {
        cout << instructions[i] << endl;
    }*/
    
    int index = 0;
    
    while(index < instructions.size()){
        switch(firstLetters[index]){
            case 'f':
                break;
            case 'b':
                break;
            case 'l':
                break;
            case 'r':
                break;
            case 'm':
                break;
            case 'p':
                int lastBracketIndex = indexLastBracket(index);
                break;
            case 'c':
                break;
            case 'i':
                break;
            case 'w':
                break;
            case 's':
                break;
            default:
                cout << "There is an error with the input" << endl;
                break;
        }
        index++;
    }
    
    
    
    
    return 0;
}


