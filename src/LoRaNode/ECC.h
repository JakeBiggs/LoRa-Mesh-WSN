#include <cstdint>
#include <iostream>
#include <array>
#include <cstring>
#include "esp_system.h"


// Curve25519 prime: p = 2^255 - 19
constexpr uint8_t P[32] = {
    0xED, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F};

// Base point (x = 9)
constexpr uint8_t BASE_POINT[32] = {9};

// Function declarations
void clamp_private_key(uint8_t private_key[32]);
void copy32(uint8_t dest[32], const uint8_t src[32]);
bool is_equal(const uint8_t a[32], const uint8_t b[32]);
void mod_inv(uint8_t res[32], const uint8_t a[32], const uint8_t p[32]);
void add_mod_p(uint8_t res[32], const uint8_t a[32], const uint8_t b[32]);
void sub_mod_p(uint8_t res[32], const uint8_t a[32], const uint8_t b[32]);
void mul_mod_p(uint8_t res[32], const uint8_t a[32], const uint8_t b[32]);
void scalar_mult(uint8_t result[32], const uint8_t scalar[32], const uint8_t point[32]);

// Clamps private key according to Curve25519 spec
void clamp_private_key(uint8_t private_key[32]) {
    private_key[0] &= 248;   // Clear last 3 bits
    private_key[31] &= 127;  // Clear highest bit
    private_key[31] |= 64;   // Set second-highest bit
}

// Copies a 32-byte array
void copy32(uint8_t dest[32], const uint8_t src[32]) {
    std::memcpy(dest, src, 32);
}

// Compares two 32-byte arrays
bool is_equal(const uint8_t a[32], const uint8_t b[32]) {
    return std::memcmp(a, b, 32) == 0;
}

// Computes the modular inverse of a number modulo p using Fermat's Little Theorem
void mod_inv(uint8_t res[32], const uint8_t a[32], const uint8_t p[32]) {
    uint8_t base[32];
    copy32(base, a);
    uint8_t exp[32] = {0xED, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                       0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                       0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                       0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7C}; // p-2
    uint8_t result[32] = {1};
    uint8_t zero[32] = {0}; // Define a zero array

    while (!is_equal(exp, zero)) {
        if (exp[0] & 1) {
            mul_mod_p(result, result, base);
        }
        mul_mod_p(base, base, base);
        for (int i = 0; i < 31; i++) {
            exp[i] = (exp[i] >> 1) | (exp[i + 1] << 7);
        }
        exp[31] >>= 1;
    }

    copy32(res, result);
}

// Adds two numbers modulo p (little-endian)
void add_mod_p(uint8_t res[32], const uint8_t a[32], const uint8_t b[32]) {
    uint16_t carry = 0;
    for (int i = 0; i < 32; i++) {
        carry += a[i] + b[i];
        res[i] = carry & 0xFF;
        carry >>= 8;
    }
    // Reduce modulo p
    while (carry) {
        carry += res[0] - P[0];
        for (int i = 0; i < 31; i++) {
            carry += res[i + 1] - P[i + 1];
            res[i] = carry & 0xFF;
            carry >>= 8;
        }
        carry += res[31] - P[31];
        res[31] = carry & 0xFF;
        carry >>= 8;
    }
}

// Subtracts two numbers modulo p
void sub_mod_p(uint8_t res[32], const uint8_t a[32], const uint8_t b[32]) {
    int16_t borrow = 0;
    for (int i = 0; i < 32; i++) {
        borrow = a[i] - b[i] - borrow;
        res[i] = borrow & 0xFF;
        borrow = (borrow < 0) ? 1 : 0;
    }
    // Reduce modulo p
    while (borrow) {
        borrow += res[0] + P[0];
        for (int i = 0; i < 31; i++) {
            borrow += res[i + 1] + P[i + 1];
            res[i] = borrow & 0xFF;
            borrow >>= 8;
        }
        borrow += res[31] + P[31];
        res[31] = borrow & 0xFF;
        borrow >>= 8;
    }
}

// Modular multiplication (Montgomery Ladder)
void mul_mod_p(uint8_t res[32], const uint8_t a[32], const uint8_t b[32]) {
    uint16_t temp[64] = {0};
    for (int i = 0; i < 32; i++) {
        uint16_t carry = 0;
        for (int j = 0; j < 32; j++) {
            carry += temp[i + j] + a[i] * b[j];
            temp[i + j] = carry & 0xFF;
            carry >>= 8;
        }
        temp[i + 32] += carry;
    }
    // Reduce modulo p
    for (int i = 0; i < 32; i++) {
        res[i] = temp[i];
    }
    for (int i = 32; i < 64; i++) {
        uint16_t carry = temp[i];
        for (int j = 0; j < 32; j++) {
            carry += res[j] - P[j];
            res[j] = carry & 0xFF;
            carry >>= 8;
        }
    }
}

// Scalar multiplication (Montgomery Ladder)
void scalar_mult(uint8_t result[32], const uint8_t scalar[32], const uint8_t point[32]) {
    uint8_t x1[32], x2[32] = {1}, z2[32] = {0}, x3[32], z3[32] = {1};
    bool swap = false;

    copy32(x1, point);
    copy32(x3, point);

    std::cout << "Starting scalar multiplication loop..." << std::endl;

    for (int i = 254; i >= 0; --i) {
        bool bit = (scalar[i / 8] >> (i % 8)) & 1;
        swap ^= bit;
        if (swap) {
            std::swap(x2, x3);
            std::swap(z2, z3);
        }
        swap = bit;

        uint8_t a[32], b[32], c[32], d[32], da[32], cb[32];

        add_mod_p(a, x2, z2);
        sub_mod_p(b, x2, z2);
        add_mod_p(c, x3, z3);
        sub_mod_p(d, x3, z3);

        mul_mod_p(da, d, a);
        mul_mod_p(cb, c, b);

        add_mod_p(a, da, cb);
        sub_mod_p(b, da, cb);
        mul_mod_p(x3, a, a);
        mul_mod_p(z3, b, b);

        mul_mod_p(a, x2, x2);
        mul_mod_p(b, z2, z2);
        sub_mod_p(c, a, b);
        add_mod_p(a, a, b);
        mul_mod_p(x2, a, c);
        mul_mod_p(z2, c, c);


    }

    if (swap) {
        std::swap(x2, x3);
        std::swap(z2, z3);
    }


    // Compute the modular inverse of z2
    uint8_t z2_inv[32];
    mod_inv(z2_inv, z2, P);

    // Multiply x2 by the modular inverse of z2
    mul_mod_p(result, x2, z2_inv);
}

// Generates a key pair
std::pair<std::array<uint8_t, 32>, std::array<uint8_t, 32>> generate_keys() {
    std::array<uint8_t, 32> private_key, public_key;

    
    for (int i = 0; i < 32; ++i) {
        private_key[i] = esp_random() & 0xFF;
    }

    
    clamp_private_key(private_key.data());    
    scalar_mult(public_key.data(), private_key.data(), BASE_POINT);

    return {private_key, public_key};
}

// Computes shared secret
std::array<uint8_t, 32> compute_shared(const std::array<uint8_t, 32>& private_key, const std::array<uint8_t, 32>& other_public_key) {
    std::array<uint8_t, 32> shared_secret;
    scalar_mult(shared_secret.data(), private_key.data(), other_public_key.data());
    return shared_secret;
}