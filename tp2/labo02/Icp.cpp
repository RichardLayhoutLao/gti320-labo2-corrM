#include "Icp.h"

#include "Matrix.h"
#include "Vector.h"
#include "Operators.h"
#include "SVD.h"

using namespace gti320;

/**
 * Calcul du déterminant 3x3
 * TODO                                                                                         // TODO DONE
 */
double gti320::determinant(const Matrix3d& M)
{
    double a11 = M(0, 0);
    double a12 = M(0, 1);
    double a13 = M(0, 2);

    double a21 = M(1, 0);
    double a22 = M(1, 1);
    double a23 = M(1, 2);

    double a31 = M(2, 0);
    double a32 = M(2, 1);
    double a33 = M(2, 2);

    double cofacteur11= (a22 * a33) - (a23 * a32);
    double cofacteur12= (a21 * a33) - (a23 * a31);
    double cofacteur13= (a21 * a32) - (a22 * a31);

    double determinant = (a11*(cofacteur11))-(a12*(cofacteur12))+(a13 * (cofacteur13));
    

    return determinant;
}

/**
 * Calcul de l'erreur entre deux nuages de points
 * TODO                                                                                         // TODO DONE
 */
double Icp::error(const Points3d& A, const Points3d& B)
{
    assert(A.cols() == B.cols());
    double erreur = 0;

    Vector<double,3> VecteurErreur;
    
    //Permet de parcourir chacune des points A et B et d'additionner l'erreur.
    for (int indexPoint = 0; indexPoint < A.cols(); indexPoint++){
        for (int index = 0; index < VecteurErreur.rows(); index++){
            VecteurErreur(index) = A(index, indexPoint) - B(index, indexPoint);
        }
        erreur += VecteurErreur.norm();
    }

    return erreur;
}


/**
 * Index du point de A qui minimise la distance à p
 * TODO                                                                                         // TODO DONE
 */
int Icp::nearestNeighbor(const Vector3d& p, const Points3d& A)
{

    Vector<double, 3> VecteurDistance;
    
    double distanceMinimal;
    double distanceMinimalPrecedant;
    int indexMinimal = 0;

    for (int indexPoint = 0; indexPoint < A.cols(); indexPoint++) {
        VecteurDistance(0) = p(0) - A(0, indexPoint);
        VecteurDistance(1) = p(1) - A(1, indexPoint);
        VecteurDistance(2) = p(2) - A(2, indexPoint);
        
        distanceMinimal = VecteurDistance.norm();
        distanceMinimalPrecedant = VecteurDistance.norm();

        if (distanceMinimal< distanceMinimalPrecedant){
            indexMinimal = indexMinimal;
        }
        distanceMinimalPrecedant = distanceMinimal;
    }

    return 0;
}

/**
 * Meilleure transformation rigide pour amener les points de B sur ceux A
 * TODO                                                                                         // TODO
 */
Matrix4d Icp::bestFitTransform(const Points3d& A, const Points3d& B)
{
    Matrix4d T;
    T.setIdentity();

    
    return T;
}
