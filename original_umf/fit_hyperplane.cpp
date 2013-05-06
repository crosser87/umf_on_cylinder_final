#include "fit_hyperplane.h"
#include <math.h>
#include <algorithm>
#include <vector>
#include <iostream>
#include <float.h>
#include <eigen2/Eigen/QR>

/**
 * using eigen2 quaternion class
 */


e::Quaternion<double> diagonalizer(e::Matrix<double, 3, 3> A, e::Matrix<double, 3, 3> &Q, e::Matrix<double, 3, 3> &D)
{
	//A must be a symmetrical matrix, otherwise we would have to use inverse instead of transpose!
	//returns a quaternion such that its matrix can be used to diagonalize A
	//in eigen decomposition we are searching for: A = Q * D * Q^T, where D = Q^T * A * Q
	// !!!NOTE: for row major the multiplication is in the opposite order, so we are using D = Q * A * Q^T ;)


    const double thetaMax = std::sqrt(FLT_MAX);

	const int maxSteps = 24; // won't need that many probably
	const int epsilon = 1e-6; // precision
    e::Quaternion<double> q(1, 0, 0, 0); //keep it normalized
	for(int i = 0; i < maxSteps; i++)
	{
		Q = q.toRotationMatrix();//
		// for q = q_0 + i* q_1 + j * q_2 + k* q_3
		// same as q = w + i * x + j * y + k * v
		// i,j,k imaginary
		//
		// | q_0^2 + q_1^2 - q_2^2 - q_3^2   2*(q_1*q_2 - q_0*q_3)           2*(q_0*q_2 + q_1*q_3)         |
		// | 2*(q_1*q_2 + q_0*q_3)           q_0^2 - q_1^2 + q_2^2 - q_3^2   2*(q_2*q_3 - q_0*q_1)         |
		// | 2*(q_1*q_3 - q_0*q_2)           2*(q_0*q_1 + q_2*q_3)           q_0^2 - q_1^2 - q_2^2 + q_3^2 |


		D = Q * A * Q.transpose();

		e::Vector3d offdiag( D(1 /*row */, 2 /*col*/), D(0, 2), D(0, 1)); //elements off the diagonale
		e::Vector3d om = offdiag.cwise().abs(); //element (coefficicent) wise abs
		int k = ( om[0] > om[1] && om[0] > om[2]) ? 0 : (om[1] > om[2]) ? 1 : 2;
		int k1 = (k + 1) % 3;
		int k2 = (k + 2) % 3;
		if(om[k] < epsilon)
		{
			std::cout << "offdiag smaller than epsilon " << std::endl;
			break; //already diagonal if the largest is smaller than our epsilon :)
		}

        double diag2 = D(k2, k2);
        double diag1 = D(k1, k1);
        double thet = ( diag2 - diag1 )/(2*offdiag[k]);
		// theta = (a_qq - a_pp) / (2*a_pg)


        double sgn = (thet > 0.0f)? 1.0f: -1.0f;
		thet *= sgn; //use abs;
		// vv don't really understand - should look at Jacobyi Transformation of a Symmetric Matrix
        double t = sgn/(thet + ( (thet < thetaMax) ? std::sqrt(thet*thet + 1.0f): thet)); // sign(T) / (|T| + sqrt(T*T + 1))

		//as in tangent t = s/c - t = sgn(theta) / ( |theta| + sqrt(theta*theta + 1))
        //double t = sgn/(that + std::sqrt(thet*thet + 1));
		//cosinus
        double c = 1.0f/std::sqrt(t*t + 1.0f); // c = 1/(t*t + 1) t = s/c
        double s = t*c;

        e::Quaternion<double> jr(0, 0, 0, 0); //jacobi rotation for this iteration
        double *jrk = &(jr.coeffs().data()[k]);
		*jrk = sgn*std::sqrt((1.0f - c)/2.0); //using 1/2 angle identity sin^2 (a) = 1/2 (1 - cos(2a))
		*jrk *= -1.0f; // since row major
		//jr[k] should be bound between [-1,1]
		jr.w() = std::sqrt(1.0f - (*jrk)*(*jrk)); // just so we have a normalized matrix
		if(jr.w() > (1 - epsilon))
		{
			std::cout << "weight close to 1" << std::endl;
			break; //pretty precise
		}
		std::cout << "Update: " << jr.w() << " " << jr.x() << " " << jr.y() << " " << jr.z() << std::endl;
		q = q*jr; //quaternion multiplication
		// (w + ix + jy + kz)*(d + ia + jb + kc) and ij = k = -ji; jk = i = -kj; ki = j = -ik; i*i = j*j = k*k = -1
		q.normalize(); // so that w*w + x*x + y*y + z*z = 1 => q/|q| -> |q| = sqrt(w*w + ... etc
		std::cout << "Quaternion: " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
	}

	Q = q.toRotationMatrix();
	D = Q * A * Q.transpose();

	return q;
}

template<int size=3>
e::Matrix<double, size, size> multMat(e::Matrix<double, size, size> M1, e::Matrix<double, size, size> M2)
{
    e::Matrix<double, size, size> result;
    for(int row = 0; row < size; row++)
	{
        for(int col = 0; col < size; col++)
		{
            double sum = 0.f;
            for(int i = 0; i < size; i++)
			{
				sum += M1(row, i) * M2(i, col);
			}
			result(row, col) = sum;
		}
	}
	return result;
}

void jacobi_sweep(e::Matrix<double, 3, 3> A, e::Matrix<double, 3, 3> &V, std::vector<double> &d)
{
	//manual implementation without quaternions
	//http://www.mpi-hd.mpg.de/astrophysik/HEA/internal/Numerical_Recipes/f11-1.pdf

	const int maxSteps = 24; // won't need that many probably
    const int epsilon = 1e-9; // precision

    const double thetaMax = std::sqrt(FLT_MAX);

	//set eigenvectors to identity
	V.setIdentity();

    std::vector<double> b(3);
    std::vector<double> z(3, 0); //this vector will accumulate the terms of the form t * a_pq

	//set eigenvalues to the diagonal
    d = std::vector<double>(3);
	d[0] = b[0] = A(0, 0);
	d[1] = b[1] = A(1, 1);
	d[2] = b[2] = A(2, 2);
	const int N = 3;

	for(int i = 0; i < maxSteps; i++)
	{
		//compute the sum of the off-diagonal elements
        double sm = std::abs(A(0, 1)) + std::abs(A(0, 2)) + std::abs(A(1, 2));
		if(sm < epsilon)
		{
			break;
		}

		//on the first sweeps use a threshold
        double thresh = 0;
		if( i < 4)
		{
			thresh = 0.2*sm/9;
		}

		//now go through the off-diagonal points
        double t = 0; //tangent
        double c = 0; //cosine
        double s = 0;

		for(int ip = 0; ip < N-1; ip++)
		{
			for(int iq = ip+1; iq < N; iq++)
			{
                double g = abs(A(ip, iq));
				if( i > 4 && g < epsilon)
				{ //skip if after four sweeps if the off-diagonal element is small
					A(ip, iq) = 0.f;
				} else if ( g > thresh )
				{
                    double theta = 0.5f*(d[iq] - d[ip])/A(ip, iq);
                    double sgn = (theta > 0)? 1.0f : -1.0f;
					theta *= sgn;
					if(theta > thetaMax) //in case it would overflow
					{
						t = sgn*0.5f/theta;
					} else {
						t = sgn/(theta + std::sqrt(theta*theta + 1));
					}

					c = 1.0f/sqrt(1.0f + t*t);
					s = t*c;

                    double tau = s/(1.0f + c);

                    double h = t*A(ip, iq);
					z[ip] -= h;
					z[iq] += h;
					d[ip] -= h;
					d[iq] += h;
					A(ip, iq) = 0.f;

                    double g = 0.f;
					for(int j = 0; j < ip - 1; j++) // 0 <= j < p
					{
						g = A(j, ip);
						h = A(j, iq);
						A(j, ip) = g - s*(h + g*tau);
						A(j, iq) = h + s*(g - h*tau);
					}

					for(int j = ip+1; j < iq; j++) // p < j < q
					{
						g = A(ip, j);
						h = A(j, iq);
						A(ip, j) = g - s*(h + g*tau);
						A(j, iq) = h + s*(g - h*tau);
					}

					for(int j = iq+1; j < N; j++) // q < j < 3
					{
						g = A(ip, j);
						h = A(iq, j);
						A(ip, j) = g - s*(h + g*tau);
						A(iq, j) = h + s*(g - h*tau);
					}

					for(int j = 0; j < N; j++)
					{
						g = V(j, ip);
						h = V(j, iq);
						V(j, ip) = g - s*(h + g*tau);
						V(j, iq) = h + s*(g - h*tau);
					}
				} //endif threshold
			}
		}//end sweep

		for(int ip = 0; ip < N; ip++)
		{
			b[ip] += z[ip];
			d[ip] = b[ip];
			z[ip] = 0;
		}

	} // end iterations

}

/**
 * @brief tridiagonal3x3 - tridiagonal decomposition in-place for 3x3 matrix - this is the same as hessenberg decomposition of a self-adjoint matrix
 *
 * Tridiagonal means that only the main diagonal and the two subdiagonals are set, the rest is 0
 * for example:
 * | 4 1 0 |
 * | 1 4 1 |
 * | 0 1 4 |
 *
 * this function takes a matrix and decomposies it into Q * T * Q^T so that T is tridiagonal and Q is orthogonal matrix (meaning Q*Q^T = I)
 *
 *
 * @param mat - the matrix - has to be symmetrical = selfadjoint for non complex matrices; M = M^T; after finished it becomes Q
 * @param diag - T's main diagonal
 * @param subdiag - T's subdiagonal
 */
void tridiagonal3x3(e::Matrix<double, 3, 3> &mat, std::vector<double> &diag, std::vector<double> &subdiag)
{
    diag[0] = mat(0, 0);
    double v1norm2 = mat(0, 2); v1norm2 *= v1norm2;
    if(v1norm2 == 0) //if it is already in tridiagonal form
    {
        diag[1] = mat(1, 1);
        diag[2] = mat(2, 2);
        subdiag[0] = mat(0, 1);
        subdiag[1] = mat(1, 2);
        mat.setIdentity();
    } else
    { //we don't have it in tridiagonal form
        double beta = sqrt(mat(0, 1)*mat(0, 1) + v1norm2);
        double invBeta = 1.0f/beta;
        double m01 = mat(0, 1)*invBeta;
        double m02 = mat(0, 2)*invBeta;
        double q = 2*m01*mat(1,2) + m02*(mat(2,2) - mat(1,1));
        diag[1] = mat(1, 1) + m02*q;
        diag[2] = mat(2, 2) - m02*q;
        subdiag[0] = beta;
        subdiag[1] = mat(1,2) - m01*q;

        //store Q into mat
        mat(0, 0) = 1;
        mat(0, 1) = 0;
        mat(0, 2) = 0;
        mat(1, 0) = 0;
        mat(1, 1) = m01;
        mat(1, 2) = m02;
        mat(2, 0) = 0;
        mat(2, 1) = m02;
        mat(2, 2) = -m01;
    }
}

#define PRECISION 1e-9
/**
 * @brief isMuchSmallerThan - a, b should be positive
 * @param a
 * @param b
 * @return
 */
inline bool isMuchSmallerThan(double a, double b)
{
    return a <= b*PRECISION;
}

inline void givens_rotation(double a, double b, double &c, double &s)
{
    if(b == 0)
    {
        c = 1; s = 0;
    }
    else if(std::abs(b) > std::abs(a))
    {
        double t = -a/b;
        s = 1.0/std::sqrt(1.0 + t*t);
        c = s * t;
    }
    else
    {
        double t = -b/a;
        c = 1.0/std::sqrt(1 + t*t);
        s = c * t;
    }
}

void tridiagonalQRStep(std::vector<double> &diag, std::vector<double> &subdiag, int start, int end, e::Matrix<double, 3, 3> &Q)
{
    double td = (diag[end-1] - diag[end])*0.5;
    double e2 = subdiag[end-1]*subdiag[end-1];
    double mu = diag[end] - e2/(td + (td>0? 1: -1)*std::sqrt(td*td + e2));
    double x = diag[start] - mu;
    double z = subdiag[start];

    for(int k = start; k < end; ++k)
    {
        double c, s;
        givens_rotation(x, z, c, s);

        //do T = G' T G
        double sdk = s * diag[k] + c * subdiag[k];
        double dkp1 = s*subdiag[k] + c*diag[k + 1];

        diag[k] = c * (c * diag[k] - s * subdiag[k]) - s * (c * subdiag[k] - s * diag[k+1]);
        diag[k+1] = s * sdk + c * dkp1;
        subdiag[k] = c * sdk - s * dkp1;

        if (k > start)
          subdiag[k - 1] = c * subdiag[k-1] - s * z;

        x = subdiag[k];

        if (k < end - 1)
        {
          z = -s * subdiag[k+1];
          subdiag[k + 1] = c * subdiag[k+1];
        }

        //apply givens rotation
        //this only modifies two columns k and k+1

        //0
        double m_i_k = Q(0, k);
        double m_i_k1 = Q(0, k+1);
        Q(0, k) = c*m_i_k - s*m_i_k1;
        Q(0, k+1) = s*m_i_k + c*m_i_k1;

        //1
        m_i_k = Q(1, k);
        m_i_k1 = Q(1, k+1);
        Q(1, k) = c*m_i_k - s*m_i_k1;
        Q(1, k+1) = s*m_i_k + c*m_i_k1;

        //2
        m_i_k = Q(2, k);
        m_i_k1 = Q(2, k+1);
        Q(2, k) = c*m_i_k - s*m_i_k1;
        Q(2, k+1) = s*m_i_k + c*m_i_k1;

    }
}

/**
 * @brief eigenvalue/vector decomposition
 * @param matrix - the input matrix - must be symmetrical
 * @param eigenVectors - the columns store the eigenVectors
 * @param eigenValues - the eigenValues - they are unsorted
 */
void eigenSolver(e::Matrix<double, 3, 3> matrix, e::Matrix<double, 3, 3> &eigenVectors, std::vector<double> &eigenValues)
{
    //instead of sweeping through all in a given order, choose the largest off-diagonal
    //value in the rotation matrix and set it to 0

    const int N = 3; //number of columns

    eigenVectors = matrix;
    std::vector<double> &diag = eigenValues; //just an alias so it's more understandable
    std::vector<double> subdiag(2, 0);
    tridiagonal3x3(eigenVectors, diag, subdiag);

    //std::cout << "After tridiag3x3 matrix:" << std::endl;
    //std::cout << eigenVectors << std::endl;
    //std::cout << "*****" << std::endl;

    int end = N - 1;
    int start = 0;
    while(end > 0)
    {
        for(int i = start; i < end; ++i)
        {
            if(isMuchSmallerThan(std::abs(subdiag[i]), std::abs(diag[i]) + std::abs(diag[i+1])))
            {
                subdiag[i] = 0;
            }
        }

        //find the largest unreduced block
        int p = 2*((int)(subdiag[0] == 0)) + (int)(subdiag[1] == 0);
        switch(p)
        {
        case 3:
            end = 0;
            continue;
        case 2:
            start = 1;
            end = 2;
            break;
        case 1:
            start = 0;
            end = 1;
            break;
        case 0:
            start = 0;
            end = 2;
            break;
        }

        tridiagonalQRStep(diag, subdiag, start, end, eigenVectors);
        //std::cout << "Start: " << start << " End: " << end << std::endl;
        //std::cout << eigenVectors << std::endl;
        //std::cout << "********" << std::endl;
    }

}


void eigenTest()
{
    e::Matrix<double, 3, 3> Q;
    e::Matrix<double, 3, 3> corrMatrix;
    corrMatrix(0, 0) = 17.25905;
    corrMatrix(0, 1) = -23.88775;
    corrMatrix(0, 2) = 6.448684;
    corrMatrix(1, 0) = -23.88775;
    corrMatrix(1, 1) = 37.74094;
    corrMatrix(1, 2) = -0.7966833;
    corrMatrix(2, 0) = 6.448684;
    corrMatrix(2, 1) = -0.7966833;
    corrMatrix(2, 2) = 17.4851;
    std::cout << "UNITTEST" << std::endl;
    std::cout << "CorrMatrix: " << std::endl;
    std::cout << corrMatrix << std::endl;
    std::cout << "******" << std::endl;

    std::vector<double> eigenValues(3);
    //jacobi_sweep(corrMatrix, Q, eigenValues);
    eigenSolver(corrMatrix, Q, eigenValues);


    std::cout << "EigenValues: " << eigenValues[0] << " " << eigenValues[1] << " " << eigenValues[2] << std::endl;
    std::cout << "EigenVec1:" << Q(0, 0) << " " << Q(1, 0) << " " << Q(2, 0) << std::endl;
    std::cout << "EigenVec2:" << Q(0, 1) << " " << Q(1, 1) << " " << Q(2, 1) << std::endl;
    std::cout << "EigenVec3:" << Q(0, 2) << " " << Q(1, 2) << " " << Q(2, 2) << std::endl;
}


void umdFitHyperplane(int lineCount, e::Vector3d **lines, e::Hyperplane<double, 3> &hyperplane)
{
	//first compute the correlation matrix
	//should be row major
    e::Matrix<double, 3, 3> corrMatrix;
    //std::cout << "CORRELATION*******************" << std::endl;
	for(int row = 0; row < 3; row++)
	{
		for(int col = 0; col < 3; col++)
		{
			//normal matrix multiplication m * m^T, this could be probably optimized since it's symmetrical
            double sum = 0;
			for(int i = 0; i < lineCount; i++)
			{
				sum += (*(lines[i]))[row]* (*(lines[i]))[col];
			}
			corrMatrix(row, col) = sum;
            //std::cout << sum << " ";
		}
       //std::cout << std::endl;
	}
    //std::cout << "CORRELATION*******************" << std::endl;



    e::Matrix<double, 3, 3> D;
    e::Matrix<double, 3, 3> Q;
	//std::cout << "correlation matrix: " << corrMatrix << std::endl;
	//std::cout << "**********" << std::endl;
	/*
	diagonalizer(corrMatrix, Q, D); //this does basically the eigen decomposition, the quaternions rot matrix should give the eigenVectors

	//get the eigenvalues A = Q * D * QT
    std::vector<double> eigenValues(3);
    eigenValues[0] = (const double)(D(0,0));
    eigenValues[1] = (const double)(D(1,1));
    eigenValues[2] = (const double)(D(2,2));

	//find the smallest
	int k = (int)(std::min_element(eigenValues.begin(), eigenValues.end()) - eigenValues.begin());

	//get the k's row for our hyperplane normal
	hyperplane.coeffs().data()[0] = Q(k , 0);
	hyperplane.coeffs().data()[1] = Q(k, 1);
	hyperplane.coeffs().data()[2] = Q(k, 2);
	hyperplane.coeffs().data()[3] = 0; //non linear part - should go through 0s
	std::cout << "Coeffs: " << Q(k, 0) << " " << Q(k, 1) << " " << Q(k, 2) << std::endl;
	std::cout << "Q: " << Q << std::endl;
	std::cout << "******" << std::endl;
	std::cout << "D: " << D << std::endl;
	std::cout << "******" << std::endl;
	*/

    std::vector<double> eigenValues(3);
    //jacobi_sweep(corrMatrix, Q, eigenValues);
    eigenSolver(corrMatrix, Q, eigenValues);

    //eigenTest();

    //e::SelfAdjointEigenSolver< e::Matrix<double, 3, 3> > slvr(corrMatrix);
    //e::Vector3d ev = slvr.eigenvalues();
    //Q = slvr.eigenvectors();
    //eigenValues[0] = ev[0];
    //eigenValues[1] = ev[1];
    //eigenValues[2] = ev[2];

	//std::cout << "Q: " << Q << std::endl;
	//std::cout << "******" << std::endl;


	//find the smallest
	int k = (int)(std::min_element(eigenValues.begin(), eigenValues.end()) - eigenValues.begin());

	hyperplane.coeffs().data()[0] = Q(0, k);
	hyperplane.coeffs().data()[1] = Q(1, k);
	hyperplane.coeffs().data()[2] = Q(2, k);
	hyperplane.coeffs().data()[3] = 0; //non linear part - should go through 0s

	e::Vector3d hyperplaneNormal = hyperplane.normal();
	//std::cout << "Normal: " << hyperplaneNormal << std::endl;
}



/**
 * @brief eigenvalue/vector decomposition
 * @param matrix - the input matrix - must be symmetrical
 * @param eigenVectors - the columns store the eigenVectors
 * @param eigenValues - the eigenValues - they are unsorted
 */
void eigenSolver(e::Matrix<double, 2, 2> matrix, e::Matrix<double, 2, 2> &eigenVectors, std::vector<double> &eigenValues)
{
    //it's quite easy for 2 by 2 matices - use the characteristic polynom roots to get the eigenvalues
    //from that the eigenvectors are easy

    //just fancy names
    double &a = matrix(0, 0);
    double &b = matrix(0, 1);
    double &c = matrix(1, 0);
    double &d = matrix(1, 1);

    //determinant and trace
    double D = a*d - b*c;
    double T = a + d;

    //eigenvalues
    eigenValues[0] = 0.5*(T + std::sqrt(T*T - 4*D));
    eigenValues[1] = 0.5*(T - std::sqrt(T*T - 4*D));

    int index = 0;
    //sort them based on size
    if(std::abs(eigenValues[0]) < std::abs(eigenValues[1]))
    {
        index = 1;
    }

    //make use of symmetry - b == c
    if(b == 0)
    {
        eigenVectors.setIdentity();
        return;
    }

    double t = (eigenValues[index] - a)/b;
    double s= 1.0/sqrt(1 + t*t);
    eigenVectors(0, 0) = s;
    eigenVectors(0, 1) = t*s;

    t = (eigenValues[(index + 1) %2] - a)/b;
    s= 1.0/sqrt(1 + t*t);
    eigenVectors(1, 0) = s;
    eigenVectors(1, 1) = t*s;
}

void umdFitLine(int lineCount, e::Vector2d **points, Line &l)
{
    e::Vector2d mean(0, 0);
    for(int i = 0; i < lineCount; ++i)
    {
        mean += *(points[i]);
    }
    mean = mean*(1.0/lineCount);

    //move it to mean;
    for(int i = 0; i < lineCount; ++i)
    {
        *(points[i]) -= mean;
    }

    //first compute the correlation matrix
    //should be row major
    e::Matrix<double, 2, 2> corrMatrix;
    for(int row = 0; row < 2; row++)
    {
        for(int col = 0; col < 2; col++)
        {
            //normal matrix multiplication m * m^T, this could be probably optimized since it's symmetrical
            double sum = 0;
            for(int i = 0; i < lineCount; i++)
            {
                sum += (*(points[i]))[row]* (*(points[i]))[col];
            }
            corrMatrix(row, col) = sum;
        }
    }


    e::Matrix<double, 2, 2> V;

    std::vector<double> eigenValues(2);
    eigenSolver(corrMatrix, V, eigenValues);

    //we want the normal
    l.a = V(1, 0);
    l.b = V(1, 1); //the second eigenvector - it is sorted for 2x2
    //compute mean

    l.c = -l.a*mean[0] - l.b*mean[1];
}

