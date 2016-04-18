#include <iostream>

using namespace std;

typedef struct
{
    int n;
}kdtree_data;

namespace space
{
class kd_tree
{
public:
    kd_tree(kdtree_data* _data, int _n)
    {
        this->n = _n;
        this->data = _data;

        this->key_index = 0;
        this->key_value = 0;
    }
    ~kd_tree(){}

    static kd_tree* left_tree;
    static kd_tree* right_tree;

private:
    kdtree_data* data;
    int key_index;
    double key_value;
    bool is_leaf;       //true for leaf

private:
    static kd_tree* init(kdtree_data* data, int n)
    {
        kd_tree* tree;
        tree = new kd_tree(data, n);
        return tree;
    }

public:
    static kd_tree* plant(kdtree_data* data, int n)
    {

    }
    void grow();
    void climb();
    void tailor();
    void cut();

};

}
