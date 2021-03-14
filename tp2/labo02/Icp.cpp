#include "Icp.h"

#include "Matrix.h"
#include "Vector.h"
#include "Operators.h"
#include "SVD.h"

using namespace gti320;

/**
 * Calcul du déterminant 3x3
 * TODO                                                                                         // TODO
 */
double gti320::determinant(const Matrix3d& M)
{
    return 0.0;
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
 * TODO                                                                                         // TODO
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
