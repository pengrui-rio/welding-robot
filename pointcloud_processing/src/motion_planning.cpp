#include <motion_planning.h>




// void Define_StartEnd_Point(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_ptr_show, vector < vector <float> > seam_cluster_all, int seam_label, char axis, float coordinate)
// {
//   for(float i = 0; i < seam_cluster_all[seam_label].size(); i++)
//   { 
//     pcl::PointXYZRGB p;
//     p.x = cloud_ptr->points[ seam_cluster_all[seam_label][i] ].x;
//     p.y = cloud_ptr->points[ seam_cluster_all[seam_label][i] ].y;
//     p.z = cloud_ptr->points[ seam_cluster_all[seam_label][i] ].z;
//     p.b = 200;
//     p.g = 200;
//     p.r = 200;

//     cloud_ptr_show->points.push_back( p ); 
//   }
//   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   //确定起点终点
//   vector < Point3f > start_end_pointset;
//   switch (axis)
//   {
//     case 'x':
//       for(float i = 0; i < cloud_ptr_show->points.size(); i++)
//       { 
//         if( abs(cloud_ptr_show->points[ i ].x - coordinate) <= 1e-3)
//         {
//           cloud_ptr_show->points[ i ].b = 200;
//           cloud_ptr_show->points[ i ].g = 0;
//           cloud_ptr_show->points[ i ].r = 0;

//           Point3f point;
//           point.x = cloud_ptr_show->points[ i ].x;
//           point.y = cloud_ptr_show->points[ i ].y;
//           point.z = cloud_ptr_show->points[ i ].z;
//         }
//         start_end_pointset.push_back(point);
//       }
//       break;

//     case 'y':
//       for(float i = 0; i < cloud_ptr_show->points.size(); i++)
//       { 
//         if( abs(cloud_ptr_show->points[ i ].x - coordinate) <= 1e-3)
//         {
//           cloud_ptr_show->points[ i ].b = 200;
//           cloud_ptr_show->points[ i ].g = 0;
//           cloud_ptr_show->points[ i ].r = 0;

//           Point3f point;
//           point.x = cloud_ptr_show->points[ i ].x;
//           point.y = cloud_ptr_show->points[ i ].y;
//           point.z = cloud_ptr_show->points[ i ].z;
//         }
//         start_end_pointset.push_back(point);
//       }
//       break;

//     case 'z':
//       for(float i = 0; i < cloud_ptr_show->points.size(); i++)
//       { 
//         if( abs(cloud_ptr_show->points[ i ].x - coordinate) <= 1e-3)
//         {
//           cloud_ptr_show->points[ i ].b = 200;
//           cloud_ptr_show->points[ i ].g = 0;
//           cloud_ptr_show->points[ i ].r = 0;

//           Point3f point;
//           point.x = cloud_ptr_show->points[ i ].x;
//           point.y = cloud_ptr_show->points[ i ].y;
//           point.z = cloud_ptr_show->points[ i ].z;
//         }
//         start_end_pointset.push_back(point);
//       }
//       break;
    
//     default:
//       break;
//   }
//   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   //确定最终中心点：
//     //求所有的点距离之和：
//   vector < float > Point_DisSum;
//   for(float i = 0; i < seam_cloud->points.size(); i++)
//   { 
//     float point_sum = 0;
//     for(float j = 0; j < seam_cloud->points.size(); j++)
//     { 
//       point_sum += sqrt(pow( seam_cloud->points[i].x - seam_cloud->points[j].x, 2) +
//                         pow( seam_cloud->points[i].y - seam_cloud->points[j].y, 2) +
//                         pow( seam_cloud->points[i].z - seam_cloud->points[j].z, 2));
//     }
//     Point_DisSum.push_back(point_sum);
//     cout << "point_sum: " << i << " " << point_sum << endl;
//   }
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   //求Point_DisSum 最小值 
//   cout << "Point_DisSum: " << Point_DisSum.size() << endl;
//   float Point_DisSum_min = 0;
//   float Point_DisSum_index = 0;
//   for(float j = 0; j < Point_DisSum.size(); j++)
//   { 
//     if(j == 0)
//     {
//       Point_DisSum_min = Point_DisSum[0];
//     }

//     if(Point_DisSum_min > Point_DisSum[j])
//     {
//       Point_DisSum_min = Point_DisSum[j];
//       Point_DisSum_index = j;
//     }
//   }

// }



// float Distance_two_Points(pcl::PointXYZ p1, pcl::PointXYZ p2);
// vector<pcl::PointXYZ> Points_Exchange(pcl::PointXYZ p1, pcl::PointXYZ p2);
// vector <float> Compute_Segment_GeometryCenter(Cloud::Ptr cloud_ptr);
// int if_cloudSegmented_as_twoPieces(vector < float > segment_GeometryCenter_pack, Cloud::Ptr cloud_ptr, Cloud::Ptr seg_1, Cloud::Ptr seg_2, Cloud::Ptr cloud_ptr_seg, PointCloud::Ptr cloud_ptr_show, float radius);

// void Seam_Curve_Fitting(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_ptr_show, vector < vector <float> > seam_cluster_all, int seam_label)
// {
//     //1.将焊接缝的点集赋给一个点云seam_cloud
//     Cloud::Ptr seam_cloud (new Cloud);
//     for(float i = 0; i < seam_cluster_all[seam_label].size(); i++)
//     { 
//         pcl::PointXYZ ps;
//         ps.x = cloud_ptr->points[ seam_cluster_all[seam_label][i] ].x;
//         ps.y = cloud_ptr->points[ seam_cluster_all[seam_label][i] ].y;
//         ps.z = cloud_ptr->points[ seam_cluster_all[seam_label][i] ].z;
//         seam_cloud->points.push_back( ps ); 
//     }

//     //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     //1.seam_cloud all
//     Cloud::Ptr seam_cloud_transition (new Cloud);
//     Cloud::Ptr seam_cloud_seg_1 (new Cloud);
//     Cloud::Ptr seam_cloud_seg_2 (new Cloud);
//     vector < float > segment_GeometryCenter_pack0 = Compute_Segment_GeometryCenter(seam_cloud);
//     float radius = 0.001;   
//     while (ros::ok())
//     {
//         cout << "radius: " << radius << endl; 
//         int segment = if_cloudSegmented_as_twoPieces(segment_GeometryCenter_pack0, seam_cloud, seam_cloud_seg_1, seam_cloud_seg_2, seam_cloud_transition, cloud_ptr_show, radius);
//         cout << "segment: " << segment << endl;

//         if(segment < 2){radius += 0.001;}
//         else{break;}
//     }
//     pcl::PointXYZ point_8; point_8.x = segment_GeometryCenter_pack0[1]; point_8.y = segment_GeometryCenter_pack0[2]; point_8.z = segment_GeometryCenter_pack0[3];
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     //2.seam_cloud_seg_1
//     Cloud::Ptr seam_cloud_seg_11 (new Cloud);
//     Cloud::Ptr seam_cloud_seg_12 (new Cloud);
//     vector < float > segment_GeometryCenter_pack1 = Compute_Segment_GeometryCenter(seam_cloud_seg_1);
//     radius = 0.001;   
//     while (ros::ok())
//     {
//         cout << "radius: " << radius << endl; 
//         int segment = if_cloudSegmented_as_twoPieces(segment_GeometryCenter_pack1, seam_cloud_seg_1, seam_cloud_seg_11, seam_cloud_seg_12, seam_cloud_transition, cloud_ptr_show, radius);
//         cout << "segment: " << segment << endl;

//         if(segment < 2){radius += 0.001;}
//         else{break;}
//     }
//     pcl::PointXYZ point_4; point_4.x = segment_GeometryCenter_pack1[1]; point_4.y = segment_GeometryCenter_pack1[2]; point_4.z = segment_GeometryCenter_pack1[3];
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     //3.seam_cloud_seg_11
//     Cloud::Ptr seam_cloud_seg_111 (new Cloud);
//     Cloud::Ptr seam_cloud_seg_112 (new Cloud);
//     vector < float > segment_GeometryCenter_pack11 = Compute_Segment_GeometryCenter(seam_cloud_seg_11);
//     radius = 0.001;   
//     while (ros::ok())
//     {
//         cout << "radius: " << radius << endl; 
//         int segment = if_cloudSegmented_as_twoPieces(segment_GeometryCenter_pack11, seam_cloud_seg_11, seam_cloud_seg_111, seam_cloud_seg_112, seam_cloud_transition, cloud_ptr_show, radius);
//         cout << "segment: " << segment << endl;

//         if(segment < 2){radius += 0.001;}
//         else{break;}
//     }
//     pcl::PointXYZ point_2; point_2.x = segment_GeometryCenter_pack11[1]; point_2.y = segment_GeometryCenter_pack11[2]; point_2.z = segment_GeometryCenter_pack11[3];

//     //4.seam_cloud_seg_12
//     Cloud::Ptr seam_cloud_seg_121 (new Cloud);
//     Cloud::Ptr seam_cloud_seg_122 (new Cloud);
//     vector < float > segment_GeometryCenter_pack12 = Compute_Segment_GeometryCenter(seam_cloud_seg_12);
//     radius = 0.001;   
//     while (ros::ok())
//     {
//         cout << "radius: " << radius << endl; 
//         int segment = if_cloudSegmented_as_twoPieces(segment_GeometryCenter_pack12, seam_cloud_seg_12, seam_cloud_seg_121, seam_cloud_seg_122, seam_cloud_transition, cloud_ptr_show, radius);
//         cout << "segment: " << segment << endl;

//         if(segment < 2){radius += 0.001;}
//         else{break;}
//     }
//     pcl::PointXYZ point_6; point_6.x = segment_GeometryCenter_pack12[1]; point_6.y = segment_GeometryCenter_pack12[2]; point_6.z = segment_GeometryCenter_pack12[3];
 
//     //5.确定剩下的点 1 3 5 7
//     pcl::PointXYZ point_1, point_3, point_5, point_7;
//     if(Distance_two_Points(point_2, point_8) > Distance_two_Points(point_6, point_8))
//     {
//         //正序
//         vector < float > segment_GeometryCenter_pack111 = Compute_Segment_GeometryCenter(seam_cloud_seg_111); 
//         point_1.x = segment_GeometryCenter_pack111[1]; point_1.y = segment_GeometryCenter_pack111[2]; point_1.z = segment_GeometryCenter_pack111[3];

//         vector < float > segment_GeometryCenter_pack112 = Compute_Segment_GeometryCenter(seam_cloud_seg_112); 
//         point_3.x = segment_GeometryCenter_pack112[1]; point_3.y = segment_GeometryCenter_pack112[2]; point_3.z = segment_GeometryCenter_pack112[3];

//         if(Distance_two_Points(point_1, point_4) < Distance_two_Points(point_3, point_4))
//         {
//             vector<pcl::PointXYZ> p_1_3 = Points_Exchange(point_1, point_3); 
//             point_1 = p_1_3[0]; point_3 = p_1_3[1];
//         }

//         vector < float > segment_GeometryCenter_pack121 = Compute_Segment_GeometryCenter(seam_cloud_seg_121); 
//         point_5.x = segment_GeometryCenter_pack121[1]; point_5.y = segment_GeometryCenter_pack121[2]; point_5.z = segment_GeometryCenter_pack121[3];

//         vector < float > segment_GeometryCenter_pack122 = Compute_Segment_GeometryCenter(seam_cloud_seg_122); 
//         point_7.x = segment_GeometryCenter_pack122[1]; point_7.y = segment_GeometryCenter_pack122[2]; point_7.z = segment_GeometryCenter_pack122[3];

//         if(Distance_two_Points(point_7, point_4) < Distance_two_Points(point_5, point_4))
//         {
//             vector<pcl::PointXYZ> p_5_7 = Points_Exchange(point_5, point_7); 
//             point_5 = p_5_7[0]; point_7 = p_5_7[1];
//         }
//     }
//     else
//     {
//         //反序
//         vector<pcl::PointXYZ> p_2_6 = Points_Exchange(point_2, point_6); 
//         point_2 = p_2_6[0]; point_6 = p_2_6[1];

//         vector < float > segment_GeometryCenter_pack111 = Compute_Segment_GeometryCenter(seam_cloud_seg_121); 
//         point_1.x = segment_GeometryCenter_pack111[1]; point_1.y = segment_GeometryCenter_pack111[2]; point_1.z = segment_GeometryCenter_pack111[3];

//         vector < float > segment_GeometryCenter_pack112 = Compute_Segment_GeometryCenter(seam_cloud_seg_122); 
//         point_3.x = segment_GeometryCenter_pack112[1]; point_3.y = segment_GeometryCenter_pack112[2]; point_3.z = segment_GeometryCenter_pack112[3];

//         if(Distance_two_Points(point_1, point_4) < Distance_two_Points(point_3, point_4))
//         {
//             vector<pcl::PointXYZ> p_1_3 = Points_Exchange(point_1, point_3); 
//             point_1 = p_1_3[0]; point_3 = p_1_3[1];
//         }

//         vector < float > segment_GeometryCenter_pack121 = Compute_Segment_GeometryCenter(seam_cloud_seg_111); 
//         point_5.x = segment_GeometryCenter_pack121[1]; point_5.y = segment_GeometryCenter_pack121[2]; point_5.z = segment_GeometryCenter_pack121[3];

//         vector < float > segment_GeometryCenter_pack122 = Compute_Segment_GeometryCenter(seam_cloud_seg_112); 
//         point_7.x = segment_GeometryCenter_pack122[1]; point_7.y = segment_GeometryCenter_pack122[2]; point_7.z = segment_GeometryCenter_pack122[3];

//         if(Distance_two_Points(point_7, point_4) < Distance_two_Points(point_5, point_4))
//         {
//             vector<pcl::PointXYZ> p_5_7 = Points_Exchange(point_5, point_7); 
//             point_5 = p_5_7[0]; point_7 = p_5_7[1];
//         }
//     }
//     cout << "point_1: " << point_1 << endl; 
//     cout << "point_2: " << point_2 << endl; 
//     cout << "point_3: " << point_3 << endl; 
//     cout << "point_4: " << point_4 << endl; 
//     cout << "point_5: " << point_5 << endl; 
//     cout << "point_6: " << point_6 << endl; 
//     cout << "point_7: " << point_7 << endl; 
//     cout << "point_8: " << point_8 << endl; 

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//     //6.seam_cloud_seg_2
//     Cloud::Ptr seam_cloud_seg_21 (new Cloud);
//     Cloud::Ptr seam_cloud_seg_22 (new Cloud);
//     vector < float > segment_GeometryCenter_pack2 = Compute_Segment_GeometryCenter(seam_cloud_seg_2);
//     radius = 0.001;   
//     while (ros::ok())
//     {
//         cout << "radius: " << radius << endl; 
//         int segment = if_cloudSegmented_as_twoPieces(segment_GeometryCenter_pack2, seam_cloud_seg_2, seam_cloud_seg_21, seam_cloud_seg_22, seam_cloud_transition, cloud_ptr_show, radius);
//         cout << "segment: " << segment << endl;

//         if(segment < 2){radius += 0.001;}
//         else{break;}
//     }
//     pcl::PointXYZ point_12; point_12.x = segment_GeometryCenter_pack2[1]; point_12.y = segment_GeometryCenter_pack2[2]; point_12.z = segment_GeometryCenter_pack2[3];
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     //7.seam_cloud_seg_21
//     Cloud::Ptr seam_cloud_seg_211 (new Cloud);
//     Cloud::Ptr seam_cloud_seg_212 (new Cloud);
//     vector < float > segment_GeometryCenter_pack21 = Compute_Segment_GeometryCenter(seam_cloud_seg_21);
//     radius = 0.001;   
//     while (ros::ok())
//     {
//         cout << "radius: " << radius << endl; 
//         int segment = if_cloudSegmented_as_twoPieces(segment_GeometryCenter_pack21, seam_cloud_seg_21, seam_cloud_seg_211, seam_cloud_seg_212, seam_cloud_transition, cloud_ptr_show, radius);
//         cout << "segment: " << segment << endl;

//         if(segment < 2){radius += 0.001;}
//         else{break;}
//     }
//     pcl::PointXYZ point_10; point_10.x = segment_GeometryCenter_pack21[1]; point_10.y = segment_GeometryCenter_pack21[2]; point_10.z = segment_GeometryCenter_pack21[3];

//     //8.seam_cloud_seg_22
//     Cloud::Ptr seam_cloud_seg_221 (new Cloud);
//     Cloud::Ptr seam_cloud_seg_222 (new Cloud);
//     vector < float > segment_GeometryCenter_pack22 = Compute_Segment_GeometryCenter(seam_cloud_seg_22);
//     radius = 0.001;   
//     while (ros::ok())
//     {
//         cout << "radius: " << radius << endl; 
//         int segment = if_cloudSegmented_as_twoPieces(segment_GeometryCenter_pack22, seam_cloud_seg_22, seam_cloud_seg_221, seam_cloud_seg_222, seam_cloud_transition, cloud_ptr_show, radius);
//         cout << "segment: " << segment << endl;

//         if(segment < 2){radius += 0.001;}
//         else{break;}
//     }
//     pcl::PointXYZ point_14; point_14.x = segment_GeometryCenter_pack22[1]; point_14.y = segment_GeometryCenter_pack22[2]; point_14.z = segment_GeometryCenter_pack22[3];


//     //8.确定剩下的点 15 13 11 9
//     pcl::PointXYZ point_15, point_13, point_11, point_9;
//     if(Distance_two_Points(point_14, point_8) > Distance_two_Points(point_10, point_8))
//     {
//         //正序
//         vector < float > segment_GeometryCenter_pack221 = Compute_Segment_GeometryCenter(seam_cloud_seg_221); 
//         point_15.x = segment_GeometryCenter_pack221[1]; point_15.y = segment_GeometryCenter_pack221[2]; point_15.z = segment_GeometryCenter_pack221[3];

//         vector < float > segment_GeometryCenter_pack222 = Compute_Segment_GeometryCenter(seam_cloud_seg_222); 
//         point_13.x = segment_GeometryCenter_pack222[1]; point_13.y = segment_GeometryCenter_pack222[2]; point_13.z = segment_GeometryCenter_pack222[3];

//         if(Distance_two_Points(point_15, point_12) < Distance_two_Points(point_13, point_12))
//         {
//             vector<pcl::PointXYZ> p_15_13 = Points_Exchange(point_15, point_13); 
//             point_15 = p_15_13[0]; point_13 = p_15_13[1];
//         }

//         vector < float > segment_GeometryCenter_pack211 = Compute_Segment_GeometryCenter(seam_cloud_seg_211); 
//         point_11.x = segment_GeometryCenter_pack211[1]; point_11.y = segment_GeometryCenter_pack211[2]; point_11.z = segment_GeometryCenter_pack211[3];

//         vector < float > segment_GeometryCenter_pack212 = Compute_Segment_GeometryCenter(seam_cloud_seg_212); 
//         point_9.x = segment_GeometryCenter_pack212[1]; point_9.y = segment_GeometryCenter_pack212[2]; point_9.z = segment_GeometryCenter_pack212[3];

//         if(Distance_two_Points(point_9, point_12) < Distance_two_Points(point_11, point_12))
//         {
//             vector<pcl::PointXYZ> p_9_11 = Points_Exchange(point_9, point_11); 
//             point_9 = p_9_11[0]; point_11 = p_9_11[1];
//         }
//     }
//     else
//     {
//         //反序
//         vector<pcl::PointXYZ> p_14_10 = Points_Exchange(point_14, point_10); 
//         point_14 = p_14_10[0]; point_10 = p_14_10[1];

//         vector < float > segment_GeometryCenter_pack211 = Compute_Segment_GeometryCenter(seam_cloud_seg_211); 
//         point_15.x = segment_GeometryCenter_pack211[1]; point_15.y = segment_GeometryCenter_pack211[2]; point_15.z = segment_GeometryCenter_pack211[3];

//         vector < float > segment_GeometryCenter_pack212 = Compute_Segment_GeometryCenter(seam_cloud_seg_212); 
//         point_13.x = segment_GeometryCenter_pack212[1]; point_13.y = segment_GeometryCenter_pack212[2]; point_13.z = segment_GeometryCenter_pack212[3];

//         if(Distance_two_Points(point_15, point_12) < Distance_two_Points(point_13, point_12))
//         {
//             vector<pcl::PointXYZ> p_15_13 = Points_Exchange(point_15, point_13); 
//             point_15 = p_15_13[0]; point_13 = p_15_13[1];
//         }

//         vector < float > segment_GeometryCenter_pack221 = Compute_Segment_GeometryCenter(seam_cloud_seg_221); 
//         point_11.x = segment_GeometryCenter_pack221[1]; point_11.y = segment_GeometryCenter_pack221[2]; point_11.z = segment_GeometryCenter_pack221[3];

//         vector < float > segment_GeometryCenter_pack222 = Compute_Segment_GeometryCenter(seam_cloud_seg_222); 
//         point_9.x = segment_GeometryCenter_pack222[1]; point_9.y = segment_GeometryCenter_pack222[2]; point_9.z = segment_GeometryCenter_pack222[3];

//         if(Distance_two_Points(point_9, point_12) < Distance_two_Points(point_11, point_12))
//         {
//             vector<pcl::PointXYZ> p_9_11 = Points_Exchange(point_9, point_11); 
//             point_9 = p_9_11[0]; point_11 = p_9_11[1];
//         }
//     }
//     cout << "point_1: " << point_1 << endl; 
//     cout << "point_2: " << point_2 << endl; 
//     cout << "point_3: " << point_3 << endl; 
//     cout << "point_4: " << point_4 << endl; 
//     cout << "point_5: " << point_5 << endl; 
//     cout << "point_6: " << point_6 << endl; 
//     cout << "point_7: " << point_7 << endl; 
//     // cout << "point_8: " << point_8 << endl; 
//     cout << "point_8: "  << point_8  << endl; 
//     cout << "point_9: "  << point_9  << endl; 
//     cout << "point_10: " << point_10 << endl; 
//     cout << "point_11: " << point_11 << endl; 
//     cout << "point_12: " << point_12 << endl; 
//     cout << "point_13: " << point_13 << endl; 
//     cout << "point_14: " << point_14 << endl; 
//     cout << "point_15: " << point_15 << endl; 

//     cloud_ptr_show->clear();
//     for(float i = 0; i < seam_cloud->points.size(); i++)
//     { 
//         pcl::PointXYZRGB p;
//         p.x = cloud_ptr->points[ seam_cluster_all[0][i] ].x;
//         p.y = cloud_ptr->points[ seam_cluster_all[0][i] ].y;
//         p.z = cloud_ptr->points[ seam_cluster_all[0][i] ].z;
//         p.b = 200;
//         p.g = 200;
//         p.r = 200;

//         cloud_ptr_show->points.push_back( p ); 
//     }
//     pcl::PointXYZRGB p;
//     p.x = point_1.x;p.y = point_1.y;p.z = point_1.z; p.b = 0;p.b = 0;p.b = 0; cloud_ptr_show->points.push_back( p );
//     p.x = point_2.x;p.y = point_2.y;p.z = point_2.z; p.b = 0;p.b = 0;p.b = 0; cloud_ptr_show->points.push_back( p );
//     p.x = point_3.x;p.y = point_3.y;p.z = point_3.z; p.b = 0;p.b = 0;p.b = 0; cloud_ptr_show->points.push_back( p );
//     p.x = point_4.x;p.y = point_4.y;p.z = point_4.z; p.b = 0;p.b = 0;p.b = 0; cloud_ptr_show->points.push_back( p );
//     p.x = point_5.x;p.y = point_5.y;p.z = point_5.z; p.b = 0;p.b = 0;p.b = 0; cloud_ptr_show->points.push_back( p );
//     p.x = point_6.x;p.y = point_6.y;p.z = point_6.z; p.b = 0;p.b = 0;p.b = 0; cloud_ptr_show->points.push_back( p );
//     p.x = point_7.x;p.y = point_7.y;p.z = point_7.z; p.b = 0;p.b = 0;p.b = 0; cloud_ptr_show->points.push_back( p );
//     p.x = point_8.x;p.y = point_8.y;p.z = point_8.z; p.b = 0;p.b = 0;p.b = 0; cloud_ptr_show->points.push_back( p );
//     p.x = point_9.x;p.y = point_9.y;p.z = point_9.z; p.b = 0;p.b = 0;p.b = 0; cloud_ptr_show->points.push_back( p );
//     p.x = point_10.x;p.y = point_10.y;p.z = point_10.z; p.b = 0;p.b = 0;p.b = 0; cloud_ptr_show->points.push_back( p );
//     p.x = point_11.x;p.y = point_11.y;p.z = point_11.z; p.b = 0;p.b = 0;p.b = 0; cloud_ptr_show->points.push_back( p );
//     p.x = point_12.x;p.y = point_12.y;p.z = point_12.z; p.b = 0;p.b = 0;p.b = 0; cloud_ptr_show->points.push_back( p );
//     p.x = point_13.x;p.y = point_13.y;p.z = point_13.z; p.b = 0;p.b = 0;p.b = 0; cloud_ptr_show->points.push_back( p );
//     p.x = point_14.x;p.y = point_14.y;p.z = point_14.z; p.b = 0;p.b = 0;p.b = 0; cloud_ptr_show->points.push_back( p );
//     p.x = point_15.x;p.y = point_15.y;p.z = point_15.z; p.b = 0;p.b = 0;p.b = 0; cloud_ptr_show->points.push_back( p );


// }


// int if_cloudSegmented_as_twoPieces(vector < float > segment_GeometryCenter_pack, Cloud::Ptr cloud_ptr, Cloud::Ptr seg_1, Cloud::Ptr seg_2, Cloud::Ptr cloud_ptr_seg, PointCloud::Ptr cloud_ptr_show, float radius)
// {

//     pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  // 创建一个 KdTree 对象
//     kdtree.setInputCloud (cloud_ptr);  // 将前面创建的随机点云作为 KdTree 输入
//     vector<int> pointIdxRadiusSearch; // 创建两个向量，分别存放近邻的索引值、近邻的中心距
//     vector<float> pointRadiusSquaredDistance;
//     kdtree.radiusSearch (cloud_ptr->points[segment_GeometryCenter_pack[0]], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);  //
//     //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//     cloud_ptr_seg->points.clear();
//     cloud_ptr_show->points.clear();
//     for(float i = 0; i < cloud_ptr->points.size(); i++)
//     { 
//         float equal_flag = 0;
//         for(float j = 0; j < pointIdxRadiusSearch.size(); j++)
//         {
//             if(i == pointIdxRadiusSearch[j])
//             {
//                 equal_flag = 1;
//                 break;
//             }
//         }
//         if(equal_flag)
//         {
//             continue;
//         }

//         pcl::PointXYZ p;
//         p.x = cloud_ptr->points[ i ].x;
//         p.y = cloud_ptr->points[ i ].y;
//         p.z = cloud_ptr->points[ i ].z;
//         cloud_ptr_seg->points.push_back( p ); 

//         pcl::PointXYZRGB ps; 
//         ps.x = cloud_ptr->points[ i ].x;
//         ps.y = cloud_ptr->points[ i ].y;
//         ps.z = cloud_ptr->points[ i ].z;
//         ps.b = 200;
//         ps.g = 200;
//         ps.r = 200;
//         cloud_ptr_show->points.push_back( ps ); 
//     }

//     //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr ec_tree (new pcl::search::KdTree<pcl::PointXYZ>);
//     ec_tree->setInputCloud (cloud_ptr_seg);//创建点云索引向量，用于存储实际的点云信息
//     std::vector<pcl::PointIndices> cluster_indices;
//     pcl::EuclideanClusterExtraction<pcl::PointXYZ> EC;
//     EC.setClusterTolerance (0.002); //设置近邻搜索的搜索半径为2cm
//     EC.setMinClusterSize (10);//设置一个聚类需要的最少点数目为100
//     EC.setMaxClusterSize (10000000); //设置一个聚类需要的最大点数目为25000
//     EC.setSearchMethod (ec_tree);//设置点云的搜索机制
//     EC.setInputCloud (cloud_ptr_seg);// input cloud
//     EC.extract (cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中

//   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     vector<float> seg_1_cluster, seg_2_cluster;
//     if(cluster_indices.size() == 2)
//     {
//         int count = 0;
//         for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//         {        
//             count++;
//             if(count == 1)
//             {
//                 for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//                 {
//                     seg_1_cluster.push_back(*pit);   
//                 }
//             }
//             if(count == 2)
//             {
//                 for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//                 {
//                     seg_2_cluster.push_back(*pit); 
//                 }
//             }
//         }
//     }
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     for(float i = 0; i < seg_1_cluster.size(); i++)
//     { 
//         pcl::PointXYZ p; 
//         p.x = cloud_ptr_seg->points[ seg_1_cluster[i] ].x;
//         p.y = cloud_ptr_seg->points[ seg_1_cluster[i] ].y;
//         p.z = cloud_ptr_seg->points[ seg_1_cluster[i] ].z;

//         seg_1->points.push_back( p ); 
//     }
//     for(float i = 0; i < seg_2_cluster.size(); i++)
//     { 
//         pcl::PointXYZ p; 
//         p.x = cloud_ptr_seg->points[ seg_2_cluster[i] ].x;
//         p.y = cloud_ptr_seg->points[ seg_2_cluster[i] ].y;
//         p.z = cloud_ptr_seg->points[ seg_2_cluster[i] ].z;

//         seg_2->points.push_back( p ); 
//     }

//     return cluster_indices.size();
// }



// vector <float> Compute_Segment_GeometryCenter(Cloud::Ptr cloud_ptr)
// {
//   //求所有的点距离之和：
//   vector <float> Point_DisSum;
//   for(float i = 0; i < cloud_ptr->points.size(); i++)
//   { 
//     float point_sum = 0;
//     for(float j = 0; j < cloud_ptr->points.size(); j++)
//     { 
//       point_sum += Distance_two_Points(cloud_ptr->points[i], cloud_ptr->points[j]);
//     }
//     Point_DisSum.push_back(point_sum);
//     cout << "point_sum: " << i << " " << point_sum << endl;
//   }
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   //求Point_DisSum 最小值 
//   cout << "Point_DisSum: " << Point_DisSum.size() << endl;
//   float Point_DisSum_min = 0;
//   float Point_DisSum_index = 0;
//   for(float j = 0; j < Point_DisSum.size(); j++)
//   { 
//     if(j == 0)
//     {
//       Point_DisSum_min = Point_DisSum[0];
//     }

//     if(Point_DisSum_min > Point_DisSum[j])
//     {
//       Point_DisSum_min = Point_DisSum[j];
//       Point_DisSum_index = j;
//     }
//   }
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   //显示几何中心：
//   cout << "Point_DisSum_index: " << Point_DisSum_index << endl;

//   pcl::PointXYZ Seam_GeometryCenter;
//   Seam_GeometryCenter.x = cloud_ptr->points[Point_DisSum_index].x;
//   Seam_GeometryCenter.y = cloud_ptr->points[Point_DisSum_index].y;
//   Seam_GeometryCenter.z = cloud_ptr->points[Point_DisSum_index].z;
//   cout << "Seam_GeometryCenter: " << Seam_GeometryCenter << endl << endl;

//   vector <float> Seam_GeometryCenter_pack;
//   Seam_GeometryCenter_pack.push_back(Point_DisSum_index);
//   Seam_GeometryCenter_pack.push_back(Seam_GeometryCenter.x);
//   Seam_GeometryCenter_pack.push_back(Seam_GeometryCenter.y);
//   Seam_GeometryCenter_pack.push_back(Seam_GeometryCenter.z);

//   return Seam_GeometryCenter_pack;
// }


// float Distance_two_Points(pcl::PointXYZ p1, pcl::PointXYZ p2)
// {
//     return sqrt(pow( p1.x - p2.x, 2) +
//                 pow( p1.y - p2.y, 2) +
//                 pow( p1.z - p2.z, 2));
// }

// vector<pcl::PointXYZ> Points_Exchange(pcl::PointXYZ p1, pcl::PointXYZ p2)
// {
//     pcl::PointXYZ p_temp;

//     p_temp.x = p1.x    ; p_temp.y = p1.y    ; p_temp.z = p1.z;
//     p1.x     = p2.x    ; p1.y     = p2.y    ; p1.z     = p2.z;
//     p2.x     = p_temp.x; p2.y     = p_temp.y; p2.z     = p_temp.z;

//     vector<pcl::PointXYZ> p;
//     p.push_back(p1);
//     p.push_back(p2);

//     return p;
// }







// void Segment_Weld_Seam(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_ptr_show, vector < vector <float> > seam_cluster_all, Point3f start_point, Point3f end_point)
// {
//   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//   for(float i = 0; i < seam_cluster_all[0].size(); i++)
//   { 
//     pcl::PointXYZRGB p;
//     p.x = cloud_ptr->points[ seam_cluster_all[0][i] ].x;
//     p.y = cloud_ptr->points[ seam_cluster_all[0][i] ].y;
//     p.z = cloud_ptr->points[ seam_cluster_all[0][i] ].z;
//     p.b = 200;
//     p.g = 200;
//     p.r = 200;

//     cloud_ptr_show->points.push_back( p ); 
//   }
//   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




// }





//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Cloud::Ptr Extract_Seam_edge(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_ptr_show, vector < vector <float> > seam_cluster_all, int seam_label)
{
    //将焊接缝的点集赋给一个点云seam_cloud
    Cloud::Ptr seam_cloud (new Cloud);
    for(float i = 0; i < seam_cluster_all[seam_label].size(); i++)
    { 
        pcl::PointXYZ ps;
        ps.x = cloud_ptr->points[ seam_cluster_all[seam_label][i] ].x;
        ps.y = cloud_ptr->points[ seam_cluster_all[seam_label][i] ].y;
        ps.z = cloud_ptr->points[ seam_cluster_all[seam_label][i] ].z;
        seam_cloud->points.push_back( ps ); 
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    std::cout << "points sieze is:"<< seam_cloud->points.size()<<std::endl;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ,pcl::Normal,pcl::Boundary> est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normEst;  //其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
    normEst.setInputCloud(seam_cloud);
    normEst.setSearchMethod(tree);
    // normEst.setRadiusSearch(2);  //法向估计的半径
    normEst.setKSearch(10);  //法向估计的点数
    normEst.compute(*normals);
    cout<<"normal size is "<< normals->size()<<endl;

    //normal_est.setViewPoint(0,0,0); //这个应该会使法向一致
    est.setInputCloud(seam_cloud);
    est.setInputNormals(normals);
    //  est.setAngleThreshold(90);
    //   est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
    est.setSearchMethod (tree);
    est.setKSearch(30);  //一般这里的数值越高，最终边界识别的精度越好
    //  est.setRadiusSearch(everagedistance);  //搜索半径
    est.compute (boundaries);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Cloud::Ptr boundPoints (new Cloud);
    int countBoundaries = 0;
    for (float i = 0; i < seam_cloud->size(); i++)
    {
        uint8_t x = (boundaries.points[i].boundary_point);
        int a = static_cast<int>(x); //该函数的功能是强制类型转换

        if ( a )
        {
            (*boundPoints).push_back(seam_cloud->points[i]);

            countBoundaries++;
        }
    }    
    cout<< "boudary size is：" << countBoundaries << endl;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    pcl::search::KdTree<pcl::PointXYZ>::Ptr ec_tree (new pcl::search::KdTree<pcl::PointXYZ>);
    ec_tree->setInputCloud (boundPoints);//创建点云索引向量，用于存储实际的点云信息
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> EC;
    EC.setClusterTolerance (0.002); //设置近邻搜索的搜索半径为2cm
    EC.setMinClusterSize (1);//设置一个聚类需要的最少点数目为100
    EC.setMaxClusterSize (10000000); //设置一个聚类需要的最大点数目为25000
    EC.setSearchMethod (ec_tree);//设置点云的搜索机制
    EC.setInputCloud (boundPoints);// input cloud
    EC.extract (cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中

    cout << "cluster_indices.size()" << cluster_indices.size() << endl;

    Cloud::Ptr seam_edge (new Cloud);
    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            seam_edge->points.push_back (boundPoints->points[*pit]);  
        }
        break;
    }

    for(float i = 0; i < seam_edge->points.size(); i++)
    { 
        pcl::PointXYZRGB p;
        p.x = seam_edge->points[ i ].x;
        p.y = seam_edge->points[ i ].y;
        p.z = seam_edge->points[ i ].z;
        p.b = 200;
        p.g = 200;
        p.r = 200;

        cloud_ptr_show->points.push_back( p ); 
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    cloud_ptr_show->points[500].b = 0;cloud_ptr_show->points[500].g = 0;cloud_ptr_show->points[500].r = 0;

    return seam_edge;
}



Cloud::Ptr Extract_Seam_edge(Cloud::Ptr cloud_ptr, PointCloud::Ptr cloud_ptr_show, vector < vector <float> > seam_cluster_all, int seam_label)
