#ifndef __F16AERO_H_
#define __F16AERO_H_

#include "../stdafx.h"
#include "F16AeroData.h"

namespace F16
{
	namespace AERO
	{		
		double _Cx(double alpha,double beta,double dele)
		{
		//CX0120_ALPHA1_BETA1_DH1_201.dat
			static int flag = 0;
	
			static double **X;
			static ND_INFO ndinfo ;

			int nDimension = 3; 

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19; 
				ndinfo.nPoints[2] = 5; 
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
				X[2] = dh1;
			}

			//int FILESIZE = 1900;
			double x[3];	
			x[0] = alpha;
			x[1] = beta;
			x[2] = dele;

			return interpn(X,_CxData,x,ndinfo);
		}/* End of function(...) */

		double _Cz(double alpha,double beta, double dele)
		{
		//CZ0120_ALPHA1_BETA1_DH1_301.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 3; /* alpha,beta,dele */

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	/* Alpha npoints */
				ndinfo.nPoints[1] = 19; /* Beta npoints  */
				ndinfo.nPoints[2] = 5;  /* dele npoints  */
				X = (double **) malloc(nDimension*sizeof(double*));
				X[0] = alpha1;
				X[1] = beta1;
				X[2] = dh1;
			}

			//int FILESIZE = 1900;	/* There are 1900 elements in the 20x19x5 3D array */
			double x[3];	/* Number of dimension */
			x[0] = alpha;
			x[1] = beta;
			x[2] = dele;
			return interpn(X,_CzData,x,ndinfo);
		}/* End of function(...) */

		double _Cm(double alpha,double beta,double dele)
		{
		//CM0120_ALPHA1_BETA1_DH1_101.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 3; 
			//int FILESIZE = 1900;

			/* Initialise everything when this function is called for the first time */
			if(flag==0)
			{
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19; 
				ndinfo.nPoints[2] = 5; 
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
				X[2] = dh1;
			}

			double x[3];	
			x[0] = alpha;
			x[1] = beta;
			x[2] = dele;
			return	interpn(X,_CmData,x,ndinfo);
		}/* End of function(...) */

		double _Cy(double alpha,double beta)
		{
		// CY0320_ALPHA1_BETA1_401.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			//int FILESIZE = 380;
			int nDimension = 2; 

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */ 
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19; 
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
			}

			double x[2];	
			x[0] = alpha;
			x[1] = beta;
			return	interpn(X,_CyData,x,ndinfo);
		}/* End of function(...) */

		double _Cn(double alpha, double beta, double dele)
		{
		//CN0120_ALPHA1_BETA1_DH2_501.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			//int FILESIZE = 1140;
			int nDimension = 3; 

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19;	
				ndinfo.nPoints[2] = 3;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
				X[2] = dh2;
			}

			double x[3];	
			x[0] = alpha;
			x[1] = beta;
			x[2] = dele;
			return (interpn(X,_CnData,x,ndinfo));
		}/* End of function(...) */

		double _Cl(double alpha, double beta,double dele){
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			//int FILESIZE = 1140;
			int nDimension = 3; 

			/* Initialise everything when this function is called for the first time */
			if(flag==0)
			{
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19;	
				ndinfo.nPoints[2] = 3;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
				X[2] = dh2;
			}

			double x[3];
			x[0] = alpha;
			x[1] = beta;
			x[2] = dele;
			return (interpn(X,_ClData,x,ndinfo));
		}/* End of function(...) */

		double _Cx_lef(double alpha,double beta)
		{
			//CX0820_ALPHA2_BETA1_202.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			//int FILESIZE = 266;
			int nDimension = 2; 

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 14;	
				ndinfo.nPoints[1] = 19; 
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha2;
				X[1] = beta1;
			}

			if(alpha > 45.0)
			{
				alpha = 45.0;
			}

			double x[2];	
			x[0] = alpha;
			x[1] = beta;
			return	interpn(X,_Cx_lefData,x,ndinfo);
		}/* End of function(...) */

		double _Cz_lef(double alpha,double beta){
			//CZ0820_ALPHA2_BETA1_302.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			//int FILESIZE = 266;
			int nDimension = 2; 

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */ 
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 14;	
				ndinfo.nPoints[1] = 19; 
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha2;
				X[1] = beta1;
		
			}
			if(alpha > 45.0)
			{
				alpha = 45.0;
			}

			double x[2];	
			x[0] = alpha;
			x[1] = beta;
			return	interpn(X,_Cz_lefData,x,ndinfo);
		}/* End of function(...) */

		double _Cm_lef(double alpha,double beta){
			//CM0820_ALPHA2_BETA1_102.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			//int FILESIZE = 266;
			int nDimension = 2; 

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 14;	
				ndinfo.nPoints[1] = 19; 
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha2;
				X[1] = beta1;
			}
			if(alpha > 45.0)
			{
				alpha = 45.0;
			}

			double x[2];	
			x[0] = alpha;
			x[1] = beta;
			return	interpn(X,_Cm_lefData,x,ndinfo);
		}/* End of function(...) */

		double _Cy_lef(double alpha,double beta){
			//CY0820_ALPHA2_BETA1_402.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			//int FILESIZE = 266;
			int nDimension = 2; 

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 14;	
				ndinfo.nPoints[1] = 19; 
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha2;
				X[1] = beta1;
			}
			if(alpha > 45.0)
			{
				alpha = 45.0;
			}

			double x[2];	
			x[0] = alpha;
			x[1] = beta;
			return	interpn(X,_Cy_lefData,x,ndinfo);
		}/* End of function(...) */

		double _Cn_lef(double alpha,double beta){
			//CN0820_ALPHA2_BETA1_502.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 2; 
			//int FILESIZE = 266;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 14;	
				ndinfo.nPoints[1] = 19; 
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha2;
				X[1] = beta1;
			}
			if(alpha > 45.0)
			{
				alpha = 45.0;
			}

			double x[2];	
			x[0] = alpha;
			x[1] = beta;
			return (interpn(X,_Cn_lefData,x,ndinfo));
		}/* End of function(...) */

		double _Cl_lef(double alpha,double beta)
		{
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo;
	
			int nDimension = 2; /* alpha,beta*/
			//int FILESIZE = 266;	/* There are 266 elements in the 14x19 2D array */

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 14;	/* Alpha npoints */
				ndinfo.nPoints[1] = 19; /* Beta npoints  */
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha2;
				X[1] = beta1;
			}
			if(alpha > 45.0)
			{
				alpha = 45.0;
			}

			double x[2];	/* Number of dimension */
			x[0] = alpha;
			x[1] = beta;
			return interpn(X,_Cl_lefData,x,ndinfo);
		}/* End of function(...) */

		double _CXq(double alpha)
		{
		//CX1120_ALPHA1_204.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */ 
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_CxqData,x,ndinfo));
		}/* End of function(...) */


		double _CZq(double alpha)
		{
			//CZ1120_ALPHA1_304.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_CzqData,x,ndinfo));
		}/* End of function(...) */


		double _CMq(double alpha){
			//CM1120_ALPHA1_104.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_CmqData,x,ndinfo));
		}/* End of function(...) */


		double _CYp(double alpha){
			//CY1220_ALPHA1_408.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_CypData,x,ndinfo));
		}/* End of function(...) */


		double _CYr(double alpha){
			//CY1320_ALPHA1_406.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_CyrData,x,ndinfo));
		}/* End of function(...) */


		double _CNr(double alpha){
			//CN1320_ALPHA1_506.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_CnrData,x,ndinfo));
		}/* End of function(...) */


		double _CNp(double alpha){
			//CN1220_ALPHA1_508.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_CnpData,x,ndinfo));
		}/* End of function(...) */


		double _CLp(double alpha)
		{
			//CL1220_ALPHA1_608.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			double x[1];	
			//int FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				}

			x[0] = alpha;
			return (interpn(X,_ClpData,x,ndinfo));
		}/* End of function(...) */


		double _CLr(double alpha)
		{
			//CL1320_ALPHA1_606.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_ClrData,x,ndinfo));
		}/* End of function(...) */


		double _delta_CXq_lef(double alpha)
		{
			//CX1420_ALPHA2_205.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 14;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 14;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha2;
			}
			if(alpha > 45.0)
			{
				alpha = 45.0;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_delta_CXq_lefData,x,ndinfo));
		}/* End of function(...) */


		double _delta_CYr_lef(double alpha){
			//CY1620_ALPHA2_407.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 14;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */ 
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 14;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha2;		
			}
			if(alpha > 45.0)
			{
				alpha = 45.0;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_delta_CYr_lefData,x,ndinfo));
		}/* End of function(...) */


		double _delta_CYp_lef(double alpha){
			//CY1520_ALPHA2_409.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 14;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 14;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha2;
			}
			if(alpha > 45.0)
			{
				alpha = 45.0;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_delta_CYp_lefData,x,ndinfo));
		}/* End of function(...) */


		double _delta_CZq_lef(double alpha){
			//CZ1420_ALPHA2_305.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 14;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 14;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha2;
			}
			if(alpha > 45.0)
			{
				alpha = 45.0;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_delta_CZq_lefData,x,ndinfo));
		}/* End of function(...) */


		double _delta_CLr_lef(double alpha){
			//CL1620_ALPHA2_607.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 14;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 14;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha2;
				}
			if(alpha > 45.0)
			{
				alpha = 45.0;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_delta_CLr_lefData,x,ndinfo));
		}/* End of function(...) */


		double _delta_CLp_lef(double alpha){
			//CL1520_ALPHA2_609.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 14;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 14;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha2;
			}
			if(alpha > 45.0)
			{
				alpha = 45.0;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_delta_CLp_lefData,x,ndinfo));
		}/* End of function(...) */


		double _delta_CMq_lef(double alpha){
			//CM1420_ALPHA2_105.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 14;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 14;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha2;
			}
			if(alpha > 45.0)
			{
				alpha = 45.0;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_delta_CMq_lefData,x,ndinfo));
		}/* End of function(...) */


		double _delta_CNr_lef(double alpha){
			//CN1620_ALPHA2_507.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 14;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 14;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha2;
			}
			if(alpha > 45.0)
			{
				alpha = 45.0;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_delta_CNr_lefData,x,ndinfo));
		}/* End of function(...) */


		double _delta_CNp_lef(double alpha){
			//CN1520_ALPHA2_509.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 14;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 14;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha2;
			}
			if(alpha > 45.0)
			{
				alpha = 45.0;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_delta_CNp_lefData,x,ndinfo));
		}/* End of function(...) */


		double _Cy_r30(double alpha, double beta){
			//CY0720_ALPHA1_BETA1_405.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 2; 
			//int FILESIZE = 380;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
			}

			double x[2];	
			x[0] = alpha;
			x[1] = beta;
			return (interpn(X,_Cy_r30Data,x,ndinfo));
		}/* End of function(...) */


		double _Cn_r30(double alpha, double beta)
		{
			//CN0720_ALPHA1_BETA1_503.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 2; 
			//int FILESIZE = 380;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
			}

			double x[2];	
			x[0] = alpha;
			x[1] = beta;
			return (interpn(X,_Cn_r30Data,x,ndinfo));
		}/* End of function(...) */


		double _Cl_r30(double alpha, double beta){
			//CL0720_ALPHA1_BETA1_603.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;

			int nDimension = 2; 
			//int FILESIZE = 380;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
			}

			double x[2];	
			x[0] = alpha;
			x[1] = beta;
			return (interpn(X,_Cl_r30Data,x,ndinfo));
		}/* End of function(...) */


		double _Cy_a20(double alpha, double beta){
			//CY0620_ALPHA1_BETA1_403.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 2; 
			//int FILESIZE = 380;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
			}

			double x[2];	
			x[0] = alpha;
			x[1] = beta;
			return (interpn(X,_Cy_a20Data,x,ndinfo));
		}/* End of function(...) */


		double _Cy_a20_lef(double alpha, double beta){
			//CY0920_ALPHA2_BETA1_404.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 2; 
			//int FILESIZE = 266;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */ 
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 14;	
				ndinfo.nPoints[1] = 19;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha2;
				X[1] = beta1;
			}
			if(alpha > 45.0)
			{
				alpha = 45.0;
			}

			double x[2];	
			x[0] = alpha;
			x[1] = beta;
			return (interpn(X,_Cy_a20_lefData,x,ndinfo));
		}/* End of function(...) */


		double _Cn_a20(double alpha, double beta){
			//CN0620_ALPHA1_BETA1_504.dat
			static int flag = 0;
			static double *DATA = (double*) NULL;
			static double **X;
			static ND_INFO ndinfo ;	
	
			int nDimension = 2; 
			//int FILESIZE = 380;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
			}

			double x[2];	
			x[0] = alpha;
			x[1] = beta;
			return (interpn(X,_Cn_a20Data,x,ndinfo));
		}/* End of function(...) */


		double _Cn_a20_lef(double alpha, double beta){
			//CN0920_ALPHA2_BETA1_505.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
		
			int nDimension = 2; 
			//int FILESIZE = 266;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0)
			{
				flag = 1;	/* Set to FILE_READ_TAG */ 
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 14;	
				ndinfo.nPoints[1] = 19;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha2;
				X[1] = beta1;
			}
			if(alpha > 45.0)
			{
				alpha = 45.0;
			}

			double x[2];	
			x[0] = alpha;
			x[1] = beta;
			return (interpn(X,_Cn_a20_lefData,x,ndinfo));
		}/* End of function(...) */


		double _Cl_a20(double alpha, double beta){
			//CL0620_ALPHA1_BETA1_604.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
		
			int nDimension = 2; 
			//int FILESIZE = 380;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0)
			{
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				ndinfo.nPoints[1] = 19;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
				X[1] = beta1;
			}

			double x[2];	
			x[0] = alpha;
			x[1] = beta;
			return (interpn(X,_Cl_a20Data,x,ndinfo));
		}/* End of function(...) */


		double _Cl_a20_lef(double alpha, double beta){
			//CL0920_ALPHA2_BETA1_605.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
		
			int nDimension = 2; 
			//int FILESIZE = 266;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0)
			{
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 14;	
				ndinfo.nPoints[1] = 19;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha2;
				X[1] = beta1;
			}
			if(alpha > 45.0)
			{
				alpha = 45.0;
			}

			double x[2];	
			x[0] = alpha;
			x[1] = beta;
			return (interpn(X,_Cl_a20_lefData,x,ndinfo));
		}/* End of function(...) */


		double _delta_CNbeta(double alpha){
			//CN9999_ALPHA1_brett.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_delta_CNbetaData,x,ndinfo));
		}/* End of function(...) */


		double _delta_CLbeta(double alpha){
			//CL9999_ALPHA1_brett.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
		
			int nDimension = 1; 
			//int FILESIZE = 20;	

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_delta_CLbetaData,x,ndinfo));
		}/* End of function(...) */


		double _delta_Cm(double alpha){
			//CM9999_ALPHA1_brett.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
	
			int nDimension = 1; 
			//int FILESIZE = 20;

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 20;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = alpha1;
			}

			double x[1];	
			x[0] = alpha;
			return (interpn(X,_delta_CmData,x,ndinfo));
		}/* End of function(...) */


		double _eta_el(double el){
			//ETA_DH1_brett.dat
			static int flag = 0;
			static double **X;
			static ND_INFO ndinfo ;
		
			int nDimension = 1; 
			//int FILESIZE = 5;

			/* Initialise everything when this function is called for the first time */
			if(flag==0){
				flag = 1;	/* Set to FILE_READ_TAG */
				ndinfo.nDimension = nDimension;
				ndinfo.nPoints = intVector(nDimension);
				ndinfo.nPoints[0] = 5;	
				X = (double **) malloc(nDimension*sizeof(double*));

				X[0] = dh1;
			}

			double x[1];	
			x[0] = el;
			return (interpn(X,_eta_elData,x,ndinfo));
		}/* End of function(...) */


		/*
		double _delta_Cm_ds(double alpha, double el){
		...............
		...............
		} End of function(...) */

	} // namespace AERO

	// this is temporary while sorting out the stuff in namespace..
	class F16Aero
	{
	public:
		double		Cx_total		;
		double		Cx				;
		double		Cx_delta_lef	;
		double		dXdQ			;
		double		Cxq				;
		double		Cxq_delta_lef	;
		double		Cz_total		;
		double		Cz				;
		double		Cz_delta_lef	;
		double		dZdQ			;
		double		Czq				;
		double		Czq_delta_lef	;
		double		Cm_total		;
		double		Cm				;
		double		eta_el			;
		double		Cm_delta_lef	;
		double		dMdQ			;
		double		Cmq				;
		double		Cmq_delta_lef	;
		double		Cm_delta		;
		double		Cm_delta_ds		;
		double		Cy_total		;
		double		Cy				;
		double		Cy_delta_lef	;
		double		dYdail			;
		double		Cy_delta_r30	;
		double		dYdR			;
		double		dYdP			;
		double		Cy_delta_a20	;
		double		Cy_delta_a20_lef;
		double		Cyr				;
		double		Cyr_delta_lef	;
		double		Cyp				;
		double		Cyp_delta_lef	;
		double		Cn_total		;
		double		Cn				;
		double		Cn_delta_lef	;
		double		dNdail			;
		double		Cn_delta_r30	;
		double		dNdR			;
		double		dNdP			;
		double		Cn_delta_beta	;
		double		Cn_delta_a20	;
		double		Cn_delta_a20_lef;
		double		Cnr				;
		double		Cnr_delta_lef	;
		double		Cnp				;
		double		Cnp_delta_lef	;
		double		Cl_total		;
		double		Cl				;
		double		Cl_delta_lef	;
		double		dLdail			;
		double		Cl_delta_r30	;
		double		dLdR			;
		double		dLdP			;
		double		Cl_delta_beta	;
		double		Cl_delta_a20	;
		double		Cl_delta_a20_lef;
		double		Clr				;
		double		Clr_delta_lef	;
		double		Clp				;
		double		Clp_delta_lef	;	

		F16Aero() :
			Cx_total(0),
			Cx(0),				
			Cx_delta_lef(0),	
			dXdQ(0),			
			Cxq(0),				
			Cxq_delta_lef(0),	
			Cz_total(0),		
			Cz(0),				
			Cz_delta_lef(0),	
			dZdQ(0),			
			Czq(0),				
			Czq_delta_lef(0),	
			Cm_total(0),		
			Cm(0),				
			eta_el(0),			
			Cm_delta_lef(0),	
			dMdQ(0),			
			Cmq(0),				
			Cmq_delta_lef(0),	
			Cm_delta(0),		
			Cm_delta_ds(0),		
			Cy_total(0),		
			Cy(0),				
			Cy_delta_lef(0),	
			dYdail(0),			
			Cy_delta_r30(0),	
			dYdR(0),			
			dYdP(0),			
			Cy_delta_a20(0),	
			Cy_delta_a20_lef(0),
			Cyr(0),				
			Cyr_delta_lef(0),	
			Cyp(0),				
			Cyp_delta_lef(0),	
			Cn_total(0),		
			Cn(0),				
			Cn_delta_lef(0),	
			dNdail(0),			
			Cn_delta_r30(0),	
			dNdR(0),			
			dNdP(0),			
			Cn_delta_beta(0),	
			Cn_delta_a20(0),	
			Cn_delta_a20_lef(0),
			Cnr(0),				
			Cnr_delta_lef(0),	
			Cnp(0),				
			Cnp_delta_lef(0),	
			Cl_total(0),		
			Cl(0),				
			Cl_delta_lef(0),	
			dLdail(0),			
			Cl_delta_r30(0),	
			dLdR(0),			
			dLdP(0),			
			Cl_delta_beta(0),	
			Cl_delta_a20(0),	
			Cl_delta_a20_lef(0),
			Clr(0),				
			Clr_delta_lef(0),	
			Clp(0),				
			Clp_delta_lef(0)	
		{}
		~F16Aero() {}

		void hifi_C(double alpha,double beta,double el)
		{
			Cx = AERO::_Cx(alpha,beta,el);
			Cz = AERO::_Cz(alpha,beta,el);
			Cm = AERO::_Cm(alpha,beta,el);
			Cy = AERO::_Cy(alpha,beta);
			Cn = AERO::_Cn(alpha,beta,el);
			Cl = AERO::_Cl(alpha,beta,el);
		}

		void hifi_damping(double alpha)
		{
			Cxq = AERO::_CXq(alpha);
			Cyr = AERO::_CYr(alpha);
			Cyp = AERO::_CYp(alpha);
			Czq = AERO::_CZq(alpha);
			Clr = AERO::_CLr(alpha);
			Clp = AERO::_CLp(alpha);
			Cmq = AERO::_CMq(alpha);
			Cnr = AERO::_CNr(alpha);
			Cnp = AERO::_CNp(alpha);
		}

		void hifi_C_lef(double alpha, double beta)
		{
			Cx_delta_lef = AERO::_Cx_lef(alpha,beta) - AERO::_Cx(alpha,beta,0);
			Cz_delta_lef = AERO::_Cz_lef(alpha,beta) - AERO::_Cz(alpha,beta,0);
			Cm_delta_lef = AERO::_Cm_lef(alpha,beta) - AERO::_Cm(alpha,beta,0);
			Cy_delta_lef = AERO::_Cy_lef(alpha,beta) - AERO::_Cy(alpha,beta);
			Cn_delta_lef = AERO::_Cn_lef(alpha,beta) - AERO::_Cn(alpha,beta,0);
			Cl_delta_lef = AERO::_Cl_lef(alpha,beta) - AERO::_Cl(alpha,beta,0);
		}

		void hifi_damping_lef(double alpha)
		{
			Cxq_delta_lef = AERO::_delta_CXq_lef(alpha);
			Cyr_delta_lef = AERO::_delta_CYr_lef(alpha);
			Cyp_delta_lef = AERO::_delta_CYp_lef(alpha);
			Czq_delta_lef = AERO::_delta_CZq_lef(alpha);
			Clr_delta_lef = AERO::_delta_CLr_lef(alpha);
			Clp_delta_lef = AERO::_delta_CLp_lef(alpha);
			Cmq_delta_lef = AERO::_delta_CMq_lef(alpha);
			Cnr_delta_lef = AERO::_delta_CNr_lef(alpha);
			Cnp_delta_lef = AERO::_delta_CNp_lef(alpha);
		}

		void hifi_rudder(double alpha, double beta)
		{
			Cy_delta_r30 = AERO::_Cy_r30(alpha,beta) - AERO::_Cy(alpha,beta);
			Cn_delta_r30 = AERO::_Cn_r30(alpha,beta) - AERO::_Cn(alpha,beta,0);
			Cl_delta_r30 = AERO::_Cl_r30(alpha,beta) - AERO::_Cl(alpha,beta,0);
		}

		void hifi_ailerons(double alpha, double beta)
		{
			Cy_delta_a20     = AERO::_Cy_a20(alpha,beta) - AERO::_Cy(alpha,beta);
			Cy_delta_a20_lef = AERO::_Cy_a20_lef(alpha,beta) - AERO::_Cy_lef(alpha,beta) - Cy_delta_a20;
			Cn_delta_a20     = AERO::_Cn_a20(alpha,beta) - AERO::_Cn(alpha,beta,0);
			Cn_delta_a20_lef = AERO::_Cn_a20_lef(alpha,beta) - AERO::_Cn_lef(alpha,beta) - Cn_delta_a20;
			Cl_delta_a20     = AERO::_Cl_a20(alpha,beta) - AERO::_Cl(alpha,beta,0);
			Cl_delta_a20_lef = AERO::_Cl_a20_lef(alpha,beta) - AERO::_Cl_lef(alpha,beta) - Cl_delta_a20;
		}

		void hifi_other_coeffs(double alpha, double el)
		{
			Cn_delta_beta = AERO::_delta_CNbeta(alpha);
			Cl_delta_beta = AERO::_delta_CLbeta(alpha);
			Cm_delta      = AERO::_delta_Cm(alpha);
			eta_el        = AERO::_eta_el(el);
			Cm_delta_ds   = 0;       /* ignore deep-stall regime, delta_Cm_ds = 0 */
		}

		void updateFrame()
		{
		}
	}; // class F16Aero
}

#endif // ifndef __F16AERO_H_
