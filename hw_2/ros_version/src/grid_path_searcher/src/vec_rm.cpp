#include <cstdlib>
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

int main() {
    cout << "Test vec remove" << endl;
    vector<double> x = {1.0, 2.000001, 3.0,2.000002};
    
    for(auto item: x) {
        cout  << item <<" ";
    }
    cout << "\n";
    x.erase(std::remove(x.begin(), x.end(), 2.000001), x.end());
    for(auto item: x) {
        cout << item <<" ";
    }
    cout << "\n";

    return 0;
}