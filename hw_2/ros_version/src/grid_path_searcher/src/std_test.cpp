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




    return ; 
}



int main() {
    vec_rm();
    multi_map();
}