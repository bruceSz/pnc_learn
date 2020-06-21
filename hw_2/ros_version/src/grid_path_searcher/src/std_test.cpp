#include <cstdlib>
#include <iostream>
#include <vector>
#include <algorithm>
#include <bits/stdc++.h>

using namespace std;

int vec_rm() {
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
template<class T>
void printV(const std::vector<T>& arr) {
    for(auto da: arr) {
        cout << "da_item: " << da << " " ;
    }
    cout << endl;
}

double computeDiagonalDistance(vector<double> arr) {
    if (arr.size() == 0)
        return 0.0;
    if (arr.size() == 1)
        return arr[0];

    double tmp = *(std::min_element(arr.begin(), arr.end()));
    double dim = arr.size();
    double diagonal_dis = sqrt(pow(tmp,2) * dim);

    arr.erase(std::remove(arr.begin(), arr.end(), tmp), arr.end());
    return diagonal_dis + computeDiagonalDistance(arr);

}


void multi_map() {
     // initialize container 
    multimap<int, int> mp; 
  
    // insert elements in random order 
    mp.insert({ 4, 20 });   
    mp.insert({1, 90});
    mp.insert({ 2, 30 }); 
    mp.insert({ 1, 40 }); 
    mp.insert({ 3, 60 }); 
     
    
    mp.insert({ 5, 50 }); 
  
    auto ite = mp.begin(); 
  
    cout << "The first element is: "; 
    cout << "{" << ite->first << ", "
         << ite->second << "}\n"; 
  

    cout << "\n Old The multimap is :" << " size mp: " << mp.size() << endl;
    cout << "KEY\tELEMENT\n"; 

    for (auto it: mp) { 
        cout << " item: "<< it.first
             << " " << it.second << '\n'; 
    }
    // prints the elements 

    cout << "remove smallest" << endl;
    auto erase_it = mp.erase(mp.begin());
    
    cout << "The erased removed is: " 
        << erase_it->first << " val: " << erase_it->second << endl;

    cout << "\nNew The multimap is : \n" << " size mp: " << mp.size() << endl;
    cout << "KEY\tELEMENT\n"; 

    for (auto it: mp) { 
        cout << " item: "<< it.first
             << " " << it.second << '\n'; 
    }

    cout << "remove smallest" << endl;
    erase_it = mp.erase(mp.begin());
    
    cout << "The erased removed is: " 
        << erase_it->first << " val: " << erase_it->second << endl;

    cout << "\nNew The multimap is : \n" << " size mp: " << mp.size() << endl;
    cout << "KEY\tELEMENT\n"; 

    for (auto it: mp) { 
        cout << " item: "<< it.first
             << " " << it.second << '\n'; 
    }



    std::vector<double> testn;
    testn.push_back(1);
    testn.push_back(1);
    testn.push_back(1);


    printV(testn);

    double dis = computeDiagonalDistance(testn);
    std::cout << "dis is : " << dis << "sqrt of 3 is: " << sqrt(3) << std::endl;
}




int main() {
    vec_rm();
    multi_map();
}