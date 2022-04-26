#ifndef TRANSLATE
#define TRANSLATE

#include <vector>
#include <string>
#include <iostream>


class Translate {
    public:
        Translate();
        ~Translate();

        vector<string> translate(string webCode);
        int indexLastBracket(int start, vector<char> firstLetters);
}




#endif
