#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "port.h"
#include "math.h"
#include "usart.h"

/// <summary>
        /// 计算两个圆的交点坐标
        /// </summary>
        /// <param name="A"></param>
        /// <param name="B"></param>
        /// <returns></returns>
        uint8 CircleInsect(double x1,double y1,double r1,double x2,double y2,double r2,double (*points)[2])
        {
            double Dis = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)); // 圆心距离
            if ((x1 == x2) &&(y1 == y2) && (r1 == r2)) // 两圆重合  返回计算失败
            {                                                  
              return 1;  
            } 
            if (Dis > (r1 + r2))     // 两圆相离
            {
							
							double op=Dis/(r1+r2)+0.01;
							
							double rr1=r1*op,rr2=r2*op;
							 if ((x1 == x2) && (!(y1 == y2)))  // 圆A和圆B 横坐标相等
                {
                    double x0 = x1 = x2;
                    double y0 = y1 + (y2 - y1) * (rr1 * rr1 - rr2 * rr2 + Dis * Dis) / (2.0 * Dis * Dis);
                    double Dis1 = sqrt(rr1 * rr1 - (x0 - x1) * (x0 - x1) - (y0 - y1) * (y0 - y1));
                    points[0][0] = x0 - Dis1;
                    points[0][1] = y0;
                    points[1][0] = x0 + Dis1;
                    points[1][1] = y0;
									//printf("C");
                }
                else if ((!(x1 == x2)) && (y1 == y2)) // 圆A和圆B 纵坐标相等
                {
                    double y0 = y1 = y2;
                    double x0 = x1 + (x2 - x1) * (rr1 * rr1 - rr2 * rr2 + Dis * Dis) / (2.0 * Dis * Dis);
                    double Dis1 = sqrt(rr1 * rr1 - (x0 - x1) * (x0 - x1) - (y0 - y1) * (y0 - y1));
                    points[0][0] = x0;
                    points[0][1]= y0 - Dis1;
                    points[1][0]= x0;
                    points[1][1]= y0 + Dis1;
									//printf("D");
                }
                else if ((!(x1 == x2)) && (!(y1 == y2))) // 横纵坐标都不等 
                {
                    double k1 = (y2 - y1) / (x2 - x1);//AB的斜率
                    double k2 = -1.0 / k1;             // CD的斜率
                    double x0 = x1 + (x2 - x1) * (rr1 * rr1 - rr2 * rr2 + Dis * Dis) / (2 * Dis * Dis);
                    double y0 = y1 + k1 * (x0 - x1);
                    double Dis1 = rr1 * rr1 - (x0 - x1) * (x0 - x1) - (y0 - y1) * (y0 - y1); //CE的平方
                    double Dis2 = sqrt(Dis1 / (1.0 + k2 * k2));//EF的长，过C作过E点水平直线的垂线，交于F点
                    points[0][0]=  x0 - Dis2;
                    points[0][1] = y0 + k2 * (points[0][0] - x0);
                    points[1][0] = x0 + Dis2;
                    points[1][1] = y0 + k2 * (points[1][0] - x0);
									  //printf("fuck");
                }
							
							/*
               // printf("The two circles have no intersection");
								double k1 = (y2 - y1) / (x2 - x1);//AB的斜率
								double k2 = -1.0 / k1;					
								double Dis1=sqrt((((y2 - y1)*(y2 - y1))+ ((x2 - x1)*(x2 - x1))));				
								double Dis2=((Dis1-r1-r2)/2)+((r1+r2)/2);							  
							  double b=(y1+(Dis2*(fabs(y2-y1)/Dis1)))-(x1+(k2*(fabs(Dis2*((x2-x1)/Dis1)))));
								points[0][0]=x1;
							  points[0][1]=k2*x1+b;
								points[1][0]=x2;
							  points[1][1]=k2*x2+b;
							*/
								return 0;					
            }
						if (Dis < fabs(r1 - r2))     // 两圆内含        返回计算失败
            {               
							return 1;  						
            }
            if ((Dis == (r1 + r2) ) || (Dis == fabs(r1 - r2))) // 两圆有一个交点(即两圆相切[内切和外切]) 
            {
						//	 printf("A");
                if (Dis == (r1 + r2))// 外切
                {
 
                    double op=Dis/(r1+r2)+0.01;
							
										double rr1=r1*op,rr2=r2*op;
										if ((x1 == x2) && (!(y1 == y2)))  // 圆A和圆B 横坐标相等
										{
												double x0 = x1 = x2;
												double y0 = y1 + (y2 - y1) * (rr1 * rr1 - rr2 * rr2 + Dis * Dis) / (2.0 * Dis * Dis);
												double Dis1 = sqrt(rr1 * rr1 - (x0 - x1) * (x0 - x1) - (y0 - y1) * (y0 - y1));
												points[0][0] = x0 - Dis1;
												points[0][1] = y0;
												points[1][0] = x0 + Dis1;
												points[1][1] = y0;
										//printf("C");
										}
										else if ((!(x1 == x2)) && (y1 == y2)) // 圆A和圆B 纵坐标相等
										{
												double y0 = y1 = y2;
												double x0 = x1 + (x2 - x1) * (rr1 * rr1 - rr2 * rr2 + Dis * Dis) / (2.0 * Dis * Dis);
												double Dis1 = sqrt(rr1 * rr1 - (x0 - x1) * (x0 - x1) - (y0 - y1) * (y0 - y1));
												points[0][0] = x0;
												points[0][1]= y0 - Dis1;
												points[1][0]= x0;
												points[1][1]= y0 + Dis1;
												//printf("D");
										}
										else if ((!(x1 == x2)) && (!(y1 == y2))) // 横纵坐标都不等 
										{
												double k1 = (y2 - y1) / (x2 - x1);//AB的斜率
												double k2 = -1.0 / k1;             // CD的斜率
												double x0 = x1 + (x2 - x1) * (rr1 * rr1 - rr2 * rr2 + Dis * Dis) / (2 * Dis * Dis);
												double y0 = y1 + k1 * (x0 - x1);
												double Dis1 = rr1 * rr1 - (x0 - x1) * (x0 - x1) - (y0 - y1) * (y0 - y1); //CE的平方
												double Dis2 = sqrt(Dis1 / (1.0 + k2 * k2));//EF的长，过C作过E点水平直线的垂线，交于F点
												points[0][0]=  x0 - Dis2;
												points[0][1] = y0 + k2 * (points[0][0] - x0);
												points[1][0] = x0 + Dis2;
												points[1][1] = y0 + k2 * (points[1][0] - x0);
									  //printf("fuck");
										}
								}
						else if (Dis == fabs(r1 - r2)) // 内切 (是否要考虑A包含B 还是B包含A，对结果是否有影响)
									{
                    if ((x1 == x2) &&( !(y1 == y2)))
                    {
                        if (r1 > r2) // A内含B
                        {
                            if (y1 > y2)
                            {
                                points[0][0] = x1 = x2;
                                points[0][1] = y1 - r1;
                            }
                            else
                            {
                                points[0][0] = x1 = x2;
                                points[0][1] = y1 + r1;
                            }
                        }
                        else // B 内含A
                        {
                            if (y1 > y2)
                            {
                                points[0][0] = x1 = x2;
                                points[0][1] = y1 + r1;
                            }
                            else
                            {
                                points[0][0] = x1 = x2;
                                points[0][1] = y1 - r1;
                            }
                        }
                    }
                    else if ((!(x1 == x2) )&& (y1 == y2))
                    {
                        if (r1 > r2)
                        {
                            if (x1 > x2)
                            {
                                points[0][0] = x1 - r1;
                                points[0][1] = y1 = y2;
                            }
                            else
                            {
                                points[0][0] = x1 + r1;
                                points[0][1] = y1 = y2;
                            }
                        }
                        else
                        {
                            if (x1 > x2)
                            {
                                points[0][0] = x1 + r1;
                                points[0][1] = y1 = y2;
                            }
                            else
                            {
                                points[0][0] = x1 - r1;
                                points[0][1] = y1 = y2;
                            }
                        }
                    }
                    else if ((!(x1 == x2) )&& (!(y1 == y2))) // 是否要考虑内含关系(求坐标时是否有影响)
                    {
                        // 内切情况，交点在AB连线的延长线上，要考虑切点的位置 
                        double k1 = (y2 - y1) / (x2 - x1);
                        double k2 = -1.0 / k1;
                        points[0][0] = x1 + (x1 - x2) * r1 / Dis;
                        //points[0].Y = y1 + k2 * (points[0].X - x1);
                        points[0][1] = y1 + (y1 - y2) * r1 / Dis;
                    }
                }
									return 0;
            }
            if ((Dis <( r1 + r2)) &&( Dis > fabs(r1 - r2)))    // 两圆有两个交点(内交或者外交)
            {
		//	printf("B");
                if ((x1 == x2) && (!(y1 == y2)))  // 圆A和圆B 横坐标相等
                {
                    double x0 = x1 = x2;
                    double y0 = y1 + (y2 - y1) * (r1 * r1 - r2 * r2 + Dis * Dis) / (2.0 * Dis * Dis);
                    double Dis1 = sqrt(r1 * r1 - (x0 - x1) * (x0 - x1) - (y0 - y1) * (y0 - y1));
                    points[0][0] = x0 - Dis1;
                    points[0][1] = y0;
                    points[1][0] = x0 + Dis1;
                    points[1][1] = y0;
									//printf("C");
                }
                else if ((!(x1 == x2)) && (y1 == y2)) // 圆A和圆B 纵坐标相等
                {
                    double y0 = y1 = y2;
                    double x0 = x1 + (x2 - x1) * (r1 * r1 - r2 * r2 + Dis * Dis) / (2.0 * Dis * Dis);
                    double Dis1 = sqrt(r1 * r1 - (x0 - x1) * (x0 - x1) - (y0 - y1) * (y0 - y1));
                    points[0][0] = x0;
                    points[0][1]= y0 - Dis1;
                    points[1][0]= x0;
                    points[1][1]= y0 + Dis1;
									//printf("D");
                }
                else if ((!(x1 == x2)) && (!(y1 == y2))) // 横纵坐标都不等 
                {
                    double k1 = (y2 - y1) / (x2 - x1);//AB的斜率
                    double k2 = -1.0 / k1;             // CD的斜率
                    double x0 = x1 + (x2 - x1) * (r1 * r1 - r2 * r2 + Dis * Dis) / (2 * Dis * Dis);
                    double y0 = y1 + k1 * (x0 - x1);
                    double Dis1 = r1 * r1 - (x0 - x1) * (x0 - x1) - (y0 - y1) * (y0 - y1); //CE的平方
                    double Dis2 = sqrt(Dis1 / (1.0 + k2 * k2));//EF的长，过C作过E点水平直线的垂线，交于F点
                    points[0][0]=  x0 - Dis2;
                    points[0][1] = y0 + k2 * (points[0][0] - x0);
                    points[1][0] = x0 + Dis2;
                    points[1][1] = y0 + k2 * (points[1][0] - x0);
									  //printf("fuck");
                }
            return 0;
						}   
				return 0;						
        }
/// <summary>
        /// 判断1,2直线和3,4直线的交点
        /// </summary>
        /// <param name="point1"></param>
        /// <param name="point2"></param>
        /// <param name="point3"></param>
        /// <param name="point4"></param>
        /// <returns></returns>
				
				
				
				
				
        void GetCrossPoint(double pD_x,double pD_y,double pB_x,double pB_y,double pC_x,double pC_y,double pA_x,double pA_y,double *k)
        {

 
            k[1] = (pB_x * pC_y * pD_y - pD_x * pB_y * pC_y - pA_y * pB_x * pD_y + pD_x * pB_y * pA_y + pC_x * pA_y * pD_y - pA_x * pC_y * pD_y - pC_x * pB_y * pA_y + pA_x * pB_y * pC_y) /
                (pD_y * pC_x - pA_x * pD_y - pB_y * pC_x + pA_x * pB_y + pB_x * pC_y - pD_x * pC_y - pA_y * pB_x + pA_y * pD_x);
 
            k[0] = (pD_y * (pC_x - pA_x) * (pB_x - pD_x) - pA_y * (pC_x - pA_x) * (pB_x - pD_x) + pA_x * (pC_y - pA_y) * (pB_x - pD_x) + pD_x * (pD_y - pB_y) * (pC_x - pA_x)) /
                ((pC_y - pA_y) * (pB_x - pD_x) + (pD_y - pB_y) * (pC_x - pA_x));
       
        }
				
				
				void Get_three_BS_Out_XYZ(double A_x,double A_y,double A_r, 
																	double B_x,double B_y,double B_r,
																	double C_x,double C_y,double C_r,double *PP_point_out)
				{
					static double PP_points[3][2][2]; //计算的边线缓存
					static double PP_point[210][2];   //计算点的缓存
					
					CircleInsect(A_x,A_y,A_r,B_x,B_y,B_r,PP_points[0]);
					CircleInsect(A_x,A_y,A_r,C_x,C_y,C_r,PP_points[1]);
					CircleInsect(B_x,B_y,B_r,C_x,C_y,C_r,PP_points[2]);
					
					GetCrossPoint(PP_points[0][0][0],PP_points[0][0][1], PP_points[0][1][0],PP_points[0][1][1],PP_points[1][0][0],PP_points[1][0][1],PP_points[1][1][0],PP_points[1][1][1],PP_point[0]);
					GetCrossPoint(PP_points[0][0][0],PP_points[0][0][1], PP_points[0][1][0],PP_points[0][1][1],PP_points[2][0][0],PP_points[2][0][1],PP_points[2][1][0],PP_points[2][1][1],PP_point[1]);
					GetCrossPoint(PP_points[1][0][0],PP_points[1][0][1], PP_points[1][1][0],PP_points[1][1][1],PP_points[2][0][0],PP_points[2][0][1],PP_points[2][1][0],PP_points[2][1][1],PP_point[2]);
					
	
						PP_point_out[0] = PP_point[0][0]+PP_point[1][0]+PP_point[2][0];
					  PP_point_out[1] = PP_point[0][1]+PP_point[1][1]+PP_point[2][1];
					
						PP_point_out[0] = PP_point_out[0] / 3;
            PP_point_out[1] = PP_point_out[1] / 3;
										
				}
/// <summary>
        /// 计算两基站相对于人的中心点
        /// </summary>
        /// <returns></returns>
						
				uint8 PersonPosition(double A_x,double A_y,double A_r, 
														double B_x,double B_y,double B_r,uint16 B_en,
														double C_x,double C_y,double C_r,uint16 C_en,
														double D_x,double D_y,double D_r,uint16 D_en,
														double E_x,double E_y,double E_r,uint16 E_en,
														double F_x,double F_y,double F_r,uint16 F_en,
														double G_x,double G_y,double G_r,uint16 G_en,double *point_out)
				{
					static double point[35][2];   //计算点的缓存					
					static double BS_buf[7][3];
																			 
					uint16 BS_en[7];
					
					 uint8 i=0,num=0;
					uint16 E=0,R=0,T=0;
		
					BS_buf[0][0]=A_x;BS_buf[0][1]=A_y;BS_buf[0][2]=A_r;
					BS_buf[1][0]=B_x;BS_buf[1][1]=B_y;BS_buf[1][2]=B_r;
					BS_buf[2][0]=C_x;BS_buf[2][1]=C_y;BS_buf[2][2]=C_r;
					BS_buf[3][0]=D_x;BS_buf[3][1]=D_y;BS_buf[3][2]=D_r;
					BS_buf[4][0]=E_x;BS_buf[4][1]=E_y;BS_buf[4][2]=E_r;
					BS_buf[5][0]=F_x;BS_buf[5][1]=F_y;BS_buf[5][2]=F_r;
					BS_buf[6][0]=G_x;BS_buf[6][1]=G_y;BS_buf[6][2]=G_r;
					
					BS_en[0]=1;
					BS_en[1]=B_en;
					BS_en[2]=C_en;
					BS_en[3]=D_en;
					BS_en[4]=E_en;
					BS_en[5]=F_en;
					BS_en[6]=G_en;
			
					for(E=0;E<5;E++)
					{
						
							if(BS_en[E]==1) 
							{								
									for(R=E+1;R<6;R++)
									{
											if(BS_en[R]==1) 
											{
													for(T=R+1;T<7;T++)
													{
															if(BS_en[T]==1) 
															{
																  uint8 sure_flag=0;
																  double Dist_BS_BS;
																
																  Dist_BS_BS=sqrt(pow((BS_buf[E][0]-BS_buf[R][0]),2)+pow((BS_buf[E][1]-BS_buf[R][1]),2));
																  if(((Dist_BS_BS+BS_buf[R][2])>BS_buf[E][2])&&((Dist_BS_BS+BS_buf[E][2])>BS_buf[R][2]))
																	{
																		sure_flag++;
																	}
																	
																	Dist_BS_BS=sqrt(pow((BS_buf[E][0]-BS_buf[T][0]),2)+pow((BS_buf[E][1]-BS_buf[T][1]),2));
																  if(((Dist_BS_BS+BS_buf[T][2])>BS_buf[E][2])&&((Dist_BS_BS+BS_buf[E][2])>BS_buf[T][2]))
																	{
																		sure_flag++;
																	}
																	
																	Dist_BS_BS=sqrt(pow((BS_buf[R][0]-BS_buf[T][0]),2)+pow((BS_buf[R][1]-BS_buf[T][1]),2));
																  if(((Dist_BS_BS+BS_buf[T][2])>BS_buf[R][2])&&((Dist_BS_BS+BS_buf[R][2])>BS_buf[T][2]))
																	{
																		sure_flag++;
																	}
																	if(sure_flag==3)
																	{
																	Get_three_BS_Out_XYZ(BS_buf[E][0],BS_buf[E][1],BS_buf[E][2],BS_buf[R][0],BS_buf[R][1],BS_buf[R][2],BS_buf[T][0],BS_buf[T][1],BS_buf[T][2],point[num]);
																  num++;																			
																	}						
																	else //存在内含
																	{
																		
																	}
																	
															}
													}
												
											}
										
									}
							}
					}
			
					 
					
					 if(num==0) return 1;
					
						point_out[0]=0;
						point_out[1]=0;
						for(i=0;i<num;i++)//
						{
							
							point_out[0] += point[i][0];
							point_out[1] += point[i][1];
						}
						point_out[0] = point_out[0] / num;
            point_out[1] = point_out[1] / num;
						/*
						point_out[0] = (point[0][0] + point[1][0] + point[2][0]) / 3;
            point_out[1] = (point[0][1] + point[1][1] + point[2][1]) / 3;
           */
            //返回中心坐标
							
            return 0;
					

				}
				
