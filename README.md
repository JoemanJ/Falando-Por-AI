## Explicação dos Filtros:

### 1. Filtro Passa Baixa:

```cpp
// Initialize the low-pass filter
void lpf_init(LPF *f, float cutoff_freq, float sample_rate) {
    float dt = 1.0f / sample_rate;                  // Time step
    float RC = 1.0f / (2.0f * M_PI * cutoff_freq);  // RC time constant
    f->alpha = dt / (RC + dt);                      // Alpha coefficient
    f->prev = 0.0f;                                 // Previous output sample
}
```

A função `lpf_init` é responsável por configurar e inicializar as variáveis do filtro antes que ele possa ser usado. Ela alcula o **passo de tempo (`dt`)** ou o intervalo de tempo entre duas amostras consecutivas do áudio (inverso da taxa de amostragem). A fórmula $f_c=\frac{1}{2\pi RC}$ define a frequência de corte, e o código está apenas a rearranjando para encontrar RC a partir da `cutoff_freq`. E, por fim, o passo mais importante: calcula o coeficiente de suavização **`alpha`** . Este valor, que varia entre 0 e 1, determina o quão "forte" é o filtro.

```cpp
// Apply low-pass filter
float apply_lpf(LPF *f, float x) {
    float y = f->alpha * x + (1.0f - f->alpha) * f->prev; // Apply low-pass filter formula
    f->prev = y;                                          // Update previous output sample
    return y;
}
```

`float y = f->alpha * x + (1.0f - f->alpha) * f->prev;` Esta é a **equação do filtro** em si. A nova amostra de saída (`y`) é uma **média ponderada** entre a amostra de entrada atual (`x`) e a amostra de saída anterior (`f->prev`). O coeficiente `alpha` controla o peso. A saída é uma combinação de `alpha` por cento da nova amostra e `(1 - alpha)` por cento da saída anterior.


### 2. Filtro Passa Alta:

void hpf_init(HPF *f, float cutoff_freq, float sample_rate) {
    float dt = 1.0f / sample_rate;                  // Time step
    float RC = 1.0f / (2.0f * M_PI * cutoff_freq);  // RC time constant
    f->alpha = RC / (RC + dt);                      // Alpha coefficient
    f->prev_x = 0.0f;                               // Previous input sample
    f->prev_y = 0.0f;                               // Previous output sample
}

A função `hpf_init` prepara as variáveis necessárias para o filtro, de forma muito similar à `lpf_init`.

```cpp
float apply_hpf(HPF *f, float x) {
    float y = f->alpha * (f->prev_y + x - f->prev_x);   // Apply high-pass filter formula
    f->prev_x = x;                                      // Update previous input sample
    f->prev_y = y;                                      // Update previous output sample
    return y;
}
```
E a função `apply_hpf` também funciona de maneira similar à `lpf_apply`.


### 3. Distorção:

Este código implementa um dos efeitos de distorção de áudio mais simples e conhecidos: o **hard clipping** (ou ceifamento/corte abrupto). Diferente dos outros filtros, esse filtro não precisa de uma *struct* de controle.

```cpp
float apply_distortion(float x, float threshold) {
    if (x > threshold) return threshold;    // Clamp to threshold
    if (x < -threshold) return -threshold;  // Clamp to negative threshold
    return x;
}
```
**`if (x > threshold) return threshold;`** : Se o valor da amostra de entrada (`x`) for **maior** que o limiar positivo (`threshold`), a função não retorna o valor original `x`, mas sim o próprio valor do `threshold`.

**`if (x < -threshold) return -threshold;`** : Da mesma forma, se o valor da amostra for **menor** que o limiar negativo (`-threshold`), a função retorna o valor `-threshold`.

**`return x;`** : Se a amostra estiver **dentro** dos limites (entre `-threshold` e `threshold`), ela passa pelo efeito sem nenhuma alteração.


### 4. Eco:
