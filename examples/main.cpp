#include"uskinCanDriver.h"
#include<stdio.h>
#include <vector>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

void plot_surf(std::vector <struct _single_node_reading> * reading)
{
  std::vector<std::vector<double>> x, y, z;
  for (double i = 1; i <= 6;  i++) {
      std::vector<double> x_row, y_row, z_row;
      for (double j = 4; j > 0; j--) {
          x_row.push_back(i);
          y_row.push_back(j);
          z_row.push_back(reading->back().z_data);
          printf("%d ",reading->back().z_data);
          reading->pop_back();
      }
      x.push_back(x_row);
      y.push_back(y_row);
      z.push_back(z_row);
  }
    printf("\n ");
    plt::plot_surface(x, y, z);
    plt::show();
}

void afonso_master(std::vector <struct _single_node_reading> * reading)
{
  printf("\n<----------------------------------------------->\n");
  for (double i = 1; i <= 6;  i++) {
      for (double j = 4; j > 0; j--) {
        float norm = (((float)reading->back().z_data - 18300)/ (25500 - 18300))*100;
			printf("%.0f", norm);
			if ((int)norm/10 == 0) {
				printf("     ");
			}else if ((int)norm/100==0) {
				printf("    ");
			}else{
				printf("   ");
			}
      reading->pop_back();
		}
		printf("\n");
	}
}

int main()
{

  UskinCanDriver uskin;

  struct can_filter rfilter[1];

  rfilter[0].can_id   = 0x135;
  rfilter[0].can_mask = CAN_SFF_MASK;
  //rfilter[1].can_id   = 0x101;
  //rfilter[1].can_mask = CAN_SFF_MASK;


  uskin.open_connection();
  //printf("%d", uskin.a);
  uskin.request_data(0x201);

  
  while(true)
  {
      std::vector <struct _single_node_reading> instant_reading;
//    uskin.read_data(rfilter, sizeof(rfilter));
      uskin.read_data(&instant_reading);

     /*  std::vector <std::vector <int>> matrix(2, std::vector<int>(2));
      

      for (int i = 0; i <2; i++)
      {  
        for (int j = 0; j <2; j++)
        {
          matrix[i][j] = i+j ;
        }
      }

      printf("Matrix sixe is %d \n", matrix.size());

      for (int i = 0; i <2; i++)
      {  
        for (int j = 0; j <2; j++)
        {
          printf("%d  ", matrix[i][j]);

        }
        printf( "\n");
      }

      matrix.clear();
      printf("Matrix sixe is %d \n", matrix.size());


      for (int i = 0; i <2; i++)
      {  
        for (int j = 0; j <2; j++)
        {
          printf("%d  ", matrix[i][j]);

        }
        printf( "\n");
      } */

//      plot_surf(&instant_reading);
      afonso_master(&instant_reading);

  }

  exit(0);
}
