//
//  main.cpp
//  Address
//
//  Created by Никита Лигостаев on 08.07.2021.
//

#include <iostream>
using namespace std;
int convert_decimal_to_binary(int decimal, int state);
int main()
{
    int decimal = 111; //change decimal number
    int state = 0; //change state for 8th BIT
    convert_decimal_to_binary(decimal, state);
    return 000;
}

//function of convertion decimal number to binary
int convert_decimal_to_binary(int decimal, int state)
{
    int i = 0;
    int n = 8; //volume of array (8 BIT)
    int array[n];
    int binary = 0;
    
    for(i=0; i<=7; i++)
    {
    array[i] = 0;
    }

    for(i=0; i<=7; i++)
    {
        array[i] = decimal % 2;
        decimal = decimal / 2;
        binary = binary + array[i]*pow(10,i); //variable for binary number as integer
        if(i == 7){
            array[i] = state;
        }
    }
    return 0;
}
