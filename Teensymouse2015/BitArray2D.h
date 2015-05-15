#ifndef BITARRAY2D_HPP
#define BITARRAY2D_HPP


template<int m, int n>
class BitArray2D
{
public:
    bool get(int i, int j) const
    {
        return (data[(i + j * m) / 8] >> ((i + j * m) % 8)) & 0x1;
    }

    void set(int i, int j, bool b)
    {
        data[(i + j * m) / 8] &= ~(1 << (i + j * m) % 8);
        data[(i + j * m) / 8] |=   b << (i + j * m) % 8;
    }

    void setAll(bool b)
    {
        for (int i = 0; i < (m * n + 7) / 8; ++i)
            data[i] = (b ? 0xff : 0);
    }

    unsigned char operator[](int i) const
    {
        return data[i];
    }

    unsigned char& operator[](int i)
    {
        return data[i];
    }

    int size() const
    {
        return (m * n + 7) / 8;
    }

private:
    unsigned char data[(m * n + 7) / 8] = {};
    
};

#endif // BITARRAY2D_HPP
