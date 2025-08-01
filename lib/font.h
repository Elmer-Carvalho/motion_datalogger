// Fontes para caracteres ASCII (32-126) e acentuados/especiais em 8x8 pixels
// Inclui números, letras maiúsculas/minúsculas, acentos e caracteres especiais
// Bytes invertidos verticalmente: primeiro byte é a linha inferior, último byte é a linha superior

static uint8_t font[] = {
    // 32: Espaço (vazio)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 33: ! (exclamação)
    0x00, 0x18, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18,
    // 34: " (aspas)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x36, 0x36,
    // 35: # (cerquilha)
    0x00, 0x14, 0x14, 0x7F, 0x14, 0x7F, 0x14, 0x14,
    // 36: $ (cifrão)
    0x00, 0x08, 0x3E, 0x49, 0x3E, 0x49, 0x3E, 0x08,
    // 37: % (porcentagem)
    0x00, 0x00, 0x46, 0x26, 0x10, 0x08, 0x64, 0x62,
    // 38: & (e comercial)
    0x00, 0x34, 0x48, 0x50, 0x38, 0x44, 0x44, 0x38,
    // 39: ' (apóstrofo)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x18, 0x18,
    // 40: ( (parêntese esquerdo)
    0x00, 0x0C, 0x18, 0x30, 0x30, 0x30, 0x18, 0x0C,
    // 41: ) (parêntese direito)
    0x00, 0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x18, 0x30,
    // 42: * (asterisco)
    0x00, 0x00, 0x14, 0x7F, 0x3E, 0x7F, 0x14, 0x00,
    // 43: + (mais)
    0x00, 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00,
    // 44: , (vírgula)
    0x10, 0x08, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
    // 45: - (hífen)
    0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00,
    // 46: . (ponto)
    0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 47: / (barra)
    0x00, 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02,
    // 48-57: 0-9
    0x00, 0x3C, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C, // 0
    0x00, 0x3E, 0x08, 0x08, 0x08, 0x28, 0x18, 0x08, // 1
    0x00, 0x7E, 0x40, 0x40, 0x3C, 0x02, 0x42, 0x3C, // 2
    0x00, 0x3C, 0x42, 0x02, 0x1C, 0x02, 0x42, 0x3C, // 3
    0x00, 0x04, 0x04, 0x04, 0x7C, 0x44, 0x44, 0x44, // 4
    0x00, 0x3C, 0x42, 0x02, 0x02, 0x7C, 0x40, 0x7E, // 5
    0x00, 0x3C, 0x42, 0x42, 0x7C, 0x40, 0x40, 0x3C, // 6
    0x00, 0x20, 0x20, 0x10, 0x08, 0x04, 0x02, 0x7E, // 7
    0x00, 0x3C, 0x42, 0x42, 0x3C, 0x42, 0x42, 0x3C, // 8
    0x00, 0x3C, 0x02, 0x02, 0x3E, 0x42, 0x42, 0x3C, // 9
    // 58: : (dois pontos)
    0x00, 0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00,
    // 59: ; (ponto e vírgula)
    0x10, 0x08, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00,
    // 60: < (menor que)
    0x00, 0x04, 0x08, 0x10, 0x20, 0x10, 0x08, 0x04,
    // 61: = (igual)
    0x00, 0x00, 0x00, 0x3E, 0x00, 0x3E, 0x00, 0x00,
    // 62: > (maior que)
    0x00, 0x20, 0x10, 0x08, 0x04, 0x08, 0x10, 0x20,
    // 63: ? (interrogação)
    0x00, 0x10, 0x00, 0x10, 0x1C, 0x02, 0x42, 0x3C,
    // 64: @ (arroba)
    0x00, 0x3C, 0x40, 0x4E, 0x52, 0x4E, 0x42, 0x3C,
    // 65-90: A-Z
    0x00, 0x42, 0x42, 0x7E, 0x42, 0x42, 0x24, 0x18, // A
    0x00, 0x7C, 0x42, 0x42, 0x7C, 0x42, 0x42, 0x7C, // B
    0x00, 0x3C, 0x42, 0x40, 0x40, 0x40, 0x42, 0x3C, // C
    0x00, 0x78, 0x44, 0x42, 0x42, 0x42, 0x44, 0x78, // D
    0x00, 0x7E, 0x40, 0x40, 0x7C, 0x40, 0x40, 0x7E, // E
    0x00, 0x40, 0x40, 0x40, 0x7C, 0x40, 0x40, 0x7E, // F
    0x00, 0x3C, 0x42, 0x42, 0x4E, 0x40, 0x42, 0x3C, // G
    0x00, 0x42, 0x42, 0x42, 0x7E, 0x42, 0x42, 0x42, // H
    0x00, 0x3E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x3E, // I
    0x00, 0x3C, 0x42, 0x02, 0x02, 0x02, 0x02, 0x02, // J
    0x00, 0x42, 0x44, 0x48, 0x70, 0x48, 0x44, 0x42, // K
    0x00, 0x7E, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, // L
    0x00, 0x42, 0x42, 0x42, 0x42, 0x5A, 0x66, 0x42, // M
    0x00, 0x42, 0x42, 0x46, 0x4A, 0x52, 0x62, 0x42, // N
    0x00, 0x3C, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C, // O
    0x00, 0x40, 0x40, 0x40, 0x7C, 0x42, 0x42, 0x7C, // P
    0x00, 0x3A, 0x44, 0x4A, 0x42, 0x42, 0x42, 0x3C, // Q
    0x00, 0x42, 0x44, 0x48, 0x7C, 0x42, 0x42, 0x7C, // R
    0x00, 0x3C, 0x42, 0x02, 0x3C, 0x40, 0x42, 0x3C, // S
    0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x7F, // T
    0x00, 0x3C, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, // U
    0x00, 0x18, 0x18, 0x24, 0x24, 0x42, 0x42, 0x42, // V
    0x00, 0x42, 0x66, 0x5A, 0x5A, 0x42, 0x42, 0x42, // W
    0x00, 0x42, 0x24, 0x24, 0x18, 0x24, 0x24, 0x42, // X
    0x00, 0x08, 0x08, 0x08, 0x1C, 0x22, 0x22, 0x41, // Y
    0x00, 0x7E, 0x40, 0x20, 0x18, 0x04, 0x02, 0x7E, // Z
    // 91: [ (colchete esquerdo)
    0x00, 0x3C, 0x20, 0x20, 0x20, 0x20, 0x20, 0x3C,
    // 92: \ (barra invertida)
    0x00, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
    // 93: ] (colchete direito)
    0x00, 0x3C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x3C,
    // 94: ^ (circunflexo)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x24, 0x18,
    // 95: _ (sublinhado)
    0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 96: ` (crase)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x18, 0x30,
    // 97-122: a-z
    0x00, 0x3E, 0x42, 0x3E, 0x02, 0x3C, 0x00, 0x00, // a
    0x00, 0x7C, 0x42, 0x42, 0x42, 0x7C, 0x40, 0x40, // b
    0x00, 0x3C, 0x42, 0x40, 0x42, 0x3C, 0x00, 0x00, // c
    0x00, 0x3E, 0x42, 0x42, 0x42, 0x3E, 0x02, 0x02, // d
    0x00, 0x3C, 0x40, 0x7E, 0x42, 0x3C, 0x00, 0x00, // e
    0x00, 0x10, 0x10, 0x10, 0x7C, 0x10, 0x12, 0x0C, // f
    0x7C, 0x02, 0x3E, 0x42, 0x42, 0x3E, 0x00, 0x00, // g
    0x00, 0x42, 0x42, 0x42, 0x42, 0x7C, 0x40, 0x40, // h
    0x00, 0x1C, 0x08, 0x08, 0x08, 0x18, 0x00, 0x08, // i
    0x18, 0x24, 0x04, 0x04, 0x04, 0x0C, 0x00, 0x04, // j
    0x00, 0x44, 0x48, 0x70, 0x48, 0x44, 0x40, 0x40, // k
    0x00, 0x1C, 0x08, 0x08, 0x08, 0x08, 0x08, 0x18, // l
    0x00, 0x42, 0x42, 0x42, 0x42, 0x7C, 0x00, 0x00, // m
    0x00, 0x42, 0x42, 0x42, 0x42, 0x7C, 0x00, 0x00, // n
    0x00, 0x3C, 0x42, 0x42, 0x42, 0x3C, 0x00, 0x00, // o
    0x40, 0x40, 0x7C, 0x42, 0x42, 0x7C, 0x00, 0x00, // p
    0x02, 0x02, 0x3E, 0x42, 0x42, 0x3E, 0x00, 0x00, // q
    0x00, 0x40, 0x40, 0x40, 0x42, 0x7C, 0x00, 0x00, // r
    0x00, 0x7C, 0x02, 0x3C, 0x40, 0x3E, 0x00, 0x00, // s
    0x00, 0x0C, 0x12, 0x10, 0x10, 0x7C, 0x10, 0x10, // t
    0x00, 0x3E, 0x42, 0x42, 0x42, 0x42, 0x00, 0x00, // u
    0x00, 0x18, 0x24, 0x24, 0x42, 0x42, 0x00, 0x00, // v
    0x00, 0x42, 0x66, 0x5A, 0x42, 0x42, 0x00, 0x00, // w
    0x00, 0x42, 0x24, 0x18, 0x24, 0x42, 0x00, 0x00, // x
    0x7C, 0x02, 0x3E, 0x42, 0x42, 0x42, 0x00, 0x00, // y
    0x00, 0x7E, 0x20, 0x18, 0x04, 0x7E, 0x00, 0x00, // z
    // 123: { (chave esquerda)
    0x00, 0x0C, 0x10, 0x10, 0x60, 0x10, 0x10, 0x0C,
    // 124: | (barra vertical)
    0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
    // 125: } (chave direita)
    0x00, 0x60, 0x10, 0x10, 0x0C, 0x10, 0x10, 0x60,
    // 126: ~ (til)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x32,
    // Caracteres acentuados e especiais (índices 95+)
    // 127: á
    0x00, 0x3E, 0x42, 0x3E, 0x02, 0x3C, 0x04, 0x08, // á
    // 128: â
    0x00, 0x3E, 0x42, 0x3E, 0x02, 0x3C, 0x24, 0x18, // â
    // 129: ã
    0x00, 0x3E, 0x42, 0x3E, 0x02, 0x3C, 0x4C, 0x32, // ã
    // 130: à
    0x00, 0x3E, 0x42, 0x3E, 0x02, 0x3C, 0x08, 0x30, // à
    // 131: ç
    0x18, 0x04, 0x3C, 0x40, 0x42, 0x3C, 0x00, 0x00, // ç
    // 132: é
    0x00, 0x3C, 0x40, 0x7E, 0x42, 0x3C, 0x04, 0x08, // é
    // 133: ê
    0x00, 0x3C, 0x40, 0x7E, 0x42, 0x3C, 0x24, 0x18, // ê
    // 134: í
    0x00, 0x1C, 0x08, 0x08, 0x08, 0x18, 0x04, 0x08, // í
    // 135: ó
    0x00, 0x3C, 0x42, 0x42, 0x42, 0x3C, 0x04, 0x08, // ó
    // 136: ô
    0x00, 0x3C, 0x42, 0x42, 0x42, 0x3C, 0x24, 0x18, // ô
    // 137: õ
    0x00, 0x3C, 0x42, 0x42, 0x42, 0x3C, 0x4C, 0x32, // õ
    // 138: ú
    0x00, 0x3E, 0x42, 0x42, 0x42, 0x42, 0x04, 0x08, // ú
    // 139: ü
    0x00, 0x3E, 0x42, 0x42, 0x42, 0x42, 0x24, 0x24  // ü
};

//
