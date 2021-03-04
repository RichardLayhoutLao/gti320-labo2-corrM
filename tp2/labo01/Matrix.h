#pragma once

/**
 * @file Matrix.h
 *
 * @brief Implémentation de matrices simples.
 *
 * Nom: Richard Layhout Lao
 * Code permanent : LAO19089501
 * Email : richard-layhout.lao.1@ens.etsmtl.ca
 *
 */

#include "MatrixBase.h"
#include <type_traits>

namespace gti320
{
    enum StorageType
    {
        ColumnStorage = 0,
        RowStorage = 1
    };

    // Déclaration avancée
    template <typename _Scalar, int _RowsAtCompile, int _ColsAtCompile, int _StorageType> class SubMatrix;


    /*---------------------------------------------------------------------------------------------------------------------------------------
                                                              Stockage par colonne
    ----------------------------------------------------------------------------------------------------------------------------------------*/


    /**
    /**
     * Classe Matrix spécialisé pour le cas d'un stockage par colonnes.
     *
     * (le cas d'un stockage par ligne fait l'objet d'une spécialisation de
     * patron, voir plus bas)
     */
    template <typename _Scalar = double, int _RowsAtCompile = Dynamic, int _ColsAtCompile = Dynamic, int _StorageType = ColumnStorage>
    class Matrix : public MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile> {
    public:

        /**
         * Constructeur par défaut
         */
        Matrix() : MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>() { }

        /**
         * Constructeur de copie
         */
        Matrix(const Matrix& other) : MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>(other) { }

        /**
         * Constructeur avec spécification du nombre de ligne et de colonnes
         */
        explicit Matrix(int _rows, int _cols) : MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>(_rows, _cols) { }

        /**
         * Destructeur
         */
        ~Matrix() { }

        /**
         * Opérateur de copie à partir d'une sous-matrice.
         *
         * Exemple : Matrix B = A.block(i,j,m,n);
         */
        template<typename _OtherScalar, int OtherRows, int _OtherCols, int _OtherStorage>
        Matrix& operator= (const SubMatrix<_OtherScalar, OtherRows, _OtherCols, _OtherStorage>& submatrix)
        {
            Matrix<_OtherScalar, OtherRows, _OtherCols, ColumnStorage> Matrix(submatrix.rows(), submatrix.cols());

            int total = 0;
            for (int i = 0; i < submatrix.rows(); i++) {
                for (int j = 0; j < submatrix.cols(); j++) {
                    Matrix(i, j) = submatrix(i, j);
                }
            }

            return Matrix;
        }

        /**
         * Accesseur à une entrée de la matrice (lecture seule)
         */
        _Scalar operator()(int i, int j) const
        {
            int index = (Matrix::rows() * j) + i; // colonne                                    
            return Matrix::m_storage.data()[index];
        }

        /**
         * Accesseur à une entrée de la matrice (lecture ou écriture)
         */
        _Scalar& operator()(int i, int j)
        {
            int index = (Matrix::rows() * j) + i;                                                               
            _Scalar& referenceTableauEntry = Matrix::m_storage.data()[index];
            return referenceTableauEntry;
        }

        /**                                                                                     
         * Crée une sous-matrice pour un block de taille (rows, cols) à partir
         * de l'index (i,j).
         */
        SubMatrix<_Scalar, _RowsAtCompile, _ColsAtCompile, _StorageType> block(int i, int j, int rows, int cols) const
        {
            return SubMatrix<_Scalar, _RowsAtCompile, _ColsAtCompile, _StorageType>(*this, i, j, rows, cols);
        }

        /**
         * Calcule l'inverse de la matrice
         */
        Matrix inverse() const
        {
            // Do nothing.
            return *this;
        }

        /**
         * Retourne la transposée de la matrice
         */
        template<typename _OtherScalar, int _OtherRows, int _OtherCols, int _OtherStorage>
        Matrix<_OtherScalar, _OtherRows, _OtherCols, _OtherStorage> transpose() const
        {
            Matrix<_Scalar, _OtherRows, _OtherCols, _OtherStorage> matrice(Matrix::cols(), Matrix::rows());
            
            for (int i = 0; i < matrice.rows(); i++)
            {
                for (int j = 0; j < matrice.cols(); j++)
                {
                    matrice(i, j) = (*this)(j, i);
                }
            }

            return  matrice;
        }

        /**
         * Affecte l'identité à la matrice
         */
        void setIdentity()
        {
            //Affecter la valeur 0.0 partour, sauf sur la diagonale principale où c'est 1.0.

            MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>::m_storage.setZero();

            for (int i = 0; i < MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>::m_storage.size(); i += MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>::rows()) {
                MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>::m_storage.data()[i] = 1;
                i++;
            }
        }

    };



    /*---------------------------------------------------------------------------------------------------------------------------------------
                                                              Stockage par ligne
    ----------------------------------------------------------------------------------------------------------------------------------------*/

    /**
     * Classe Matrix spécialisée pour un stockage par lignes
     */
    template <typename _Scalar, int _RowsAtCompile, int _ColsAtCompile>
    class Matrix< _Scalar, _RowsAtCompile, _ColsAtCompile, RowStorage> : public MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile> {

    public:
        /**
         * Constructeur par défaut
         */
        Matrix() : MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>() { }

        /**
         * Constructeur de copie
         */
        Matrix(const Matrix& other) : MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>(other) { }

        /**
         * Constructeur avec spécification du nombre de ligne et de colonnes
         */
        explicit Matrix(int rows, int cols) : MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>(rows, cols) { }

        /**
         * Destructeur
         */
        ~Matrix() { }

        /**
         * Opérateur de copie à partir d'une sous-matrice.
         *
         * Exemple : Matrix B = A.block(i,j,m,n);
         */
        template<typename _OtherScalar, int OtherRows, int _OtherCols, int _OtherStorage>
        Matrix& operator= (const SubMatrix<_OtherScalar, OtherRows, _OtherCols, _OtherStorage>& submatrix)
        {
            Matrix<_OtherScalar, OtherRows, _OtherCols, RowStorage> Matrix(submatrix.rows(), submatrix.cols());
            
            int total = 0;
            for (int i = 0; i < submatrix.rows(); i++) {
                for (int j = 0; j < submatrix.cols(); j++) {
                    Matrix::m_storage.data()[total] = submatrix(i, j);
                    total++;
                }
            }

            return *this;
        }

        /**
         * Accesseur à une entrée de la matrice (lecture seule)
         */
        _Scalar operator()(int i, int j) const
        {
            int index = (Matrix::cols() * i) + j; //ligne   
            return Matrix::m_storage.data()[index];
        }

        /**
         * Accesseur à une entrée de la matrice (lecture ou écriture)
         */
        _Scalar& operator()(int i, int j)
        {
            int index = (Matrix::cols() * i) + j; //ligne                                                                  
            _Scalar& referenceTableauEntry = Matrix::m_storage.data()[index];
            return referenceTableauEntry;
        }

        /**                                                                    
         * Crée une sous-matrice pour un block de taille (rows, cols) à partir
         * de l'index (i,j).
         */
        SubMatrix<_Scalar, _RowsAtCompile, _ColsAtCompile, RowStorage> block(int i, int j, int rows, int cols) const {
            return SubMatrix<_Scalar, _RowsAtCompile, _ColsAtCompile, RowStorage>(*this, i, j, rows, cols);
        }

        /**
         * Calcule l'inverse de la matrice
         */
        Matrix inverse() const
        {
            // Do nothing.
            return *this;
        }

        /**
         * Retourne la transposée de la matrice
         */
        Matrix<_Scalar, _ColsAtCompile, _RowsAtCompile, ColumnStorage> transpose() const
        {
            Matrix<_Scalar, _ColsAtCompile, _RowsAtCompile, ColumnStorage> matrice(Matrix::cols(), Matrix::rows());

            for (int j = 0; j < matrice.cols(); j++)
            {
                for (int i = 0; i < matrice.rows(); i++)
                {
                    matrice(i, j) = (*this)(j, i);
                }
            }

            return matrice;

        }






        /**
         * Affecte l'identité à la matrice
         */
        void setIdentity()
        {
            for (int i = 0; i < MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>::m_storage.size(); i += MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>::cols()) {

                MatrixBase<_Scalar, _RowsAtCompile, _ColsAtCompile>::m_storage.data()[i] = 1;
                i++;

            }
        }

    };



    /*---------------------------------------------------------------------------------------------------------------------------------------
                                                              Sous-matrice
    ----------------------------------------------------------------------------------------------------------------------------------------*/

    /**
     * Classe pour accéder à une sous-matrice.
     *
     * Un sous-matrice ne copie pas les données. Au lieu de cela, elle conserve une
     * référence à la matrice originale.
     */
    template <typename _Scalar, int _RowsAtCompile, int _ColsAtCompile, int _StorageType>
    class SubMatrix
    {
    private:
        // Référence à la matrice originale
        Matrix<_Scalar, _RowsAtCompile, _ColsAtCompile, _StorageType>& m_matrix;

        // Constructeur par défaut (privé)
        SubMatrix() {}

        // (i,j) est le coin supérieur gauche de la sous-matrice
        int m_i;        // Décalage en ligne 
        int m_j;        // Décalage en colonne

        // la sous-matrice est de dimension : m_rows x m_cols
        int m_rows;     // Height of the sub matrix (rows)
        int m_cols;     // Width of the sub matrix (columns)

    public:

        /**
         * Constructeur à partir d'une référence en lecture seule à une matrice.
         */
        SubMatrix(const Matrix<_Scalar, _RowsAtCompile, _ColsAtCompile, _StorageType>& _matrix, int _i, int _j, int _rows, int _cols) :
            m_matrix(const_cast<Matrix<_Scalar, _RowsAtCompile, _ColsAtCompile, _StorageType>&>(_matrix)),
            m_i(_i), m_j(_j), m_rows(_rows), m_cols(_cols)
        {
        }

        /**
         * Constructeur à partir d'une référence en lecture et écriture à une matrice.
         */
        explicit SubMatrix(Matrix<_Scalar, _RowsAtCompile, _ColsAtCompile, _StorageType>& _matrix    , int _i, int _j, int _rows, int _cols) :
            m_matrix(_matrix),
            m_i(_i), m_j(_j), m_rows(_rows), m_cols(_cols)
        {

        }

        /**
         * Constructeur de copie
         */
        SubMatrix(const SubMatrix& other) :
            m_matrix(other.m_matrix),
            m_i(other.m_i), m_j(other.m_j), m_rows(other.m_rows), m_cols(other.m_cols)
        {
        }

        /**
         * Destructeur
         */
        ~SubMatrix() { }

        /**
         * Opérateur de copie (à partir d'une matrice)
         *
         * Copies toutes les entrées de la matrice dans la sous-matrice.
         *
         * Note : la taille de la matrice doit correspondre à la taille de la
         * sous-matrice.
         */
        template<typename _OtherScalar, int _OtherRows, int _OtherCols, int _OtherStorage>
        SubMatrix& operator= (const Matrix<_OtherScalar, _OtherRows, _OtherCols, _OtherStorage>& matrix)
        {
            // TODO Copie les valeurs de la matrice dans la sous-matrice.                                   
            //      Note les dimensions de la matrice doivent correspondre à celle de
            //      la sous-matrice.

            for (int i = 0; i < matrix.rows(); i++){
                for (int j = 0; j < matrix.cols(); j++) {
                    (*this)(i,j) = matrix(i,j);
                }
            }

            return *this;
        }

        /**
         * Accesseur aux entrées de la sous-matrice (lecture seule)
         *
         * Note : il faut s'assurer que les indices respectent la taille de la
         * sous-matrice
         */
        _Scalar operator()(int i, int j) const
        {

            //_Scalar referenceTableauEntry = m_matrix(i + m_i, j + m_j);
            return m_matrix(i + m_i, j + m_j);
        }

        /**
         * Accesseur aux entrées de la sous-matrice (lecture et écriture)
         *
         * Note : il faut s'assurer que les indices respectent la taille de la
         * sous-matrice
         */
        _Scalar& operator()(int i, int j)
        {

            //_Scalar referenceTableauEntry = m_matrix(i + m_i, j + m_j);
            return m_matrix(i + m_i, j + m_j);
        }

        /**
         * Retourne la transposée de la sous-matrice sous la forme d'une matrice.
         */
        template<typename _OtherScalar, int _OtherRows, int _OtherCols, int _OtherStorage>
        Matrix<_OtherScalar, _OtherRows, _OtherCols, _OtherStorage> transpose() const
        {
            return m_matrix.transpose();
        }

        inline int rows() const { return m_rows; }
        inline int cols() const { return m_cols; }

    };

}
