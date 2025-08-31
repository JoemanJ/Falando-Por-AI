# Falando Por AI
## O que é
O projeto é de um modulador de voz portátil em tempo real. Ele se baseia em um microcontrolador STM32 para processamento de sinal de áudio e coordenação dos outros componentes.
Usamos um microfone de eletreto para capturar sinais de voz, o conversor ADC do próprio STM32 para digitalizar o sinal, e um módulo DAC para conversão do audio digital processado de volta em analógico. O código é um projeto padrão do STM32CubeIDE, nossa ferramenta principal de desenvovimento.

## Como usar
Para usar nosso pojeto você precisará de alguns componentes essênciais:
-  STM32F411CEU6 (STM32 Blackpill)
-  MAX9814 (Módulo Microfone Eletreto com Amplificador) 
-  PCM5102A (Módulo Conversor DAC)
  
>Outras placas STM32 com: ADC de 12 bits, timer, interface I2S podem ser usadam, mas a configuração dos periféricos tem de ser feita manualmente nesses casos. Mais detalhes na sessão [Configuração de periféricos](#config_perifericos)

Para usar nosso código recomendamos o uso do STM32CubeIDE que irá facilitar a importação do código e configuração dos perifericos do seu STM32.
Com seu STM32CubeIDE baixado, basta clonar o nosso repositório em qualquer pasta do seu armazenamento e importar o projeto usando a IDE. Para fazer isso, abra a IDE e vá em File->Import, escolha a opção General->Existing Projects into Workspace, seleciona a pasta do repositório clonado em "Select root directory" e clique em "Finish".
>Também é possível usar outras IDEs e ferramentas (Por exemplo, PlatformIO) para compilar e fazer upload do código para a placa, mas isso exige configurações adicionais que não serão explicadas aqui.

Com o código importado na sua IDE, basta conectar sua placa de desenvolvimento em um programador e upar o código para seu STM.

Em seguida, conecte os componentes conforme a imagem a seguir:

(imagem)

Por fim, alimente o seu STM32 e use um fone de ouvido na entrada p2 do seu DAC para escutar os efeitos aplicados na sua voz.

O botão conectado ao pino B1 muda para o próximo efeito de voz de forma cíclica. A ordem padrão é: Darth Vader -> Optimus Prime -> Passa baixa (voz longe) -> Passa alta (efeito de rádio) -> Low pitch shift (voz grossa) -> High pitch shift (voz aguda). 

# Como funciona

## Componentes
(pdf arrumadinho do documento "Componentes" no ClickUp)

## Processo

### (Pipeline de processamento de áudio)
### <a name="config_perifericos"></a>(Configuração dos periféricos)
(Interrupções e Double buffering)
(Diagrama do projeto do clickup)
(Explicação dos arquivos no repositório (principais nas pastas Core/\[Inc/src\])

# Filtros e clonagem de voz
A implementação dos filtros foi feita de forma que eles possam ser acoplados e desacoplados em qualquer projeto similar a este. Da forma como foi construída, as funções de filtro esperam receber amostras (um único valor por vez) sequenciais e normalizadas (*floats* entre -1.0 e 1.0) de um buffer de áudio, retornando as amostras com os filtros aplicados. Em outras palavras, seria possível, a título de exemplo, criar um programa em C usando os mesmos arquivos de filtro deste projeto (Filtro.h e Filtro.C) que leia um arquivo na extensão WAV, normalize os dados de áudio, aplique os filtros em cada amostra de dado e retorne o arquivo de áudio reconstruído, com o filtro aplicado. Uma aplicação dessa natureza pode ser vista na pasta `filter_testing`, especificamente em `filter_testing/filter_test.c`, onde experimentamos as implementações iniciais do filtro do **Darth Vader** em um arquivo de extensão WAV. 

De forma adicional, este projeto se propôs a usar uma aplicação de Inteligência Artificial (IA) que clone a voz de qualquer pessoa. Entretanto, foram encontrados desafios ao tentar embarcar o modelo de IA no hardware do projeto, de forma que esta ideia fosse invibializada, sendo um possível desafio para projetos futuros. Por consequência, a aplicação de IA foi implementada à parte, em um hardware externo (um notebook com placa de vídeo dedicada).

## Explicação dos Filtros:

### 1. Filtro Passa Baixa ⬇️:

O código a seguir implementa um **filtro passa-baixa (Low-Pass Filter - LPF)** de primeira ordem.

```cpp
// Initialize the low-pass filter
void lpf_init(LPF *f, float cutoff_freq, float sample_rate) {
    float dt = 1.0f / sample_rate;                  // Time step
    float RC = 1.0f / (2.0f * M_PI * cutoff_freq);  // RC time constant
    f->alpha = dt / (RC + dt);                      // Alpha coefficient
    f->prev = 0.0f;                                 // Previous output sample
}
```

A função `lpf_init` é responsável por configurar e inicializar as variáveis do filtro antes que ele possa ser usado. Ela alcula o **passo de tempo (`dt`)** ou o intervalo de tempo entre duas amostras consecutivas do áudio (inverso da taxa de amostragem). A fórmula $f_c=\frac{1}{2\pi RC}$ define a frequência de corte, e o código está apenas a rearranjando para encontrar RC a partir da `cutoff_freq`. E, por fim, o passo mais importante: calcula o coeficiente de suavização **`alpha`**. Este valor, que varia entre 0 e 1, determina o quão "forte" é o filtro.

```cpp
// Apply low-pass filter
float apply_lpf(LPF *f, float x) {
    float y = f->alpha * x + (1.0f - f->alpha) * f->prev; // Apply low-pass filter formula
    f->prev = y;                                          // Update previous output sample
    return y;
}
```

`float y = f->alpha * x + (1.0f - f->alpha) * f->prev;` Esta é a **equação do filtro** em si. A nova amostra de saída (`y`) é uma **média ponderada** entre a amostra de entrada atual (`x`) e a amostra de saída anterior (`f->prev`). O coeficiente `alpha` controla o peso. A saída é uma combinação de `alpha` por cento da nova amostra e `(1 - alpha)` por cento da saída anterior.

### 2. Filtro Passa Alta ⬆️:

O código a seguir implementa um **filtro passa-alta (High-Pass Filter - HPF)**.

```cpp
void hpf_init(HPF *f, float cutoff_freq, float sample_rate) {
    float dt = 1.0f / sample_rate;                  // Time step
    float RC = 1.0f / (2.0f * M_PI * cutoff_freq);  // RC time constant
    f->alpha = RC / (RC + dt);                      // Alpha coefficient
    f->prev_x = 0.0f;                               // Previous input sample
    f->prev_y = 0.0f;                               // Previous output sample
}
```

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

### 3. Distorção 〰️:

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

### 4. Eco 🗣:

O código apresentado implementa um efeito de **eco** , também conhecido como **delay digital**. Este é um dos efeitos de áudio mais fundamentais, baseado no conceito de armazenar um som e reproduzi-lo um pouco mais tarde.

```cpp
// Initialize the echo effect
// This function sets up the echo effect with a delay time and decay factor.
void echo_init(Echo* e, float delay_ms, float decay, float sample_rate) {
    e->delay_samples = (int)(sample_rate * delay_ms / 1000.0f);                     // Convert delay time to samples
    if (e->delay_samples > MAX_DELAY_SAMPLES) e->delay_samples = MAX_DELAY_SAMPLES; // Clamp to max size
    e->size = e->delay_samples;                                                     // Set size
    e->index = 0;                                                                   // Reset index  
    e->decay = decay;                                                               // Set decay factor
    memset(e->buffer, 0, sizeof(e->buffer));                                        // Clear buffer
}
```

**`e->delay_samples = (int)(sample_rate * delay_ms / 1000.0f);`**: Esta linha converte o tempo de atraso de milissegundos para o número correspondente de **amostras de áudio**. Por exemplo, com uma taxa de 44100 Hz e um delay de 500 ms, o eco precisará de `44100 * 500 / 1000 = 22050` amostras de atraso.

**`if (e->delay_samples > MAX_DELAY_SAMPLES) ...`**: Uma verificação de segurança para garantir que o tempo de atraso solicitado não exceda o tamanho máximo do *buffer* de memória alocado (`MAX_DELAY_SAMPLES`), evitando erros de acesso à memória.

**`memset(e->buffer, 0, sizeof(e->buffer));`**: Limpa completamente o *buffer* de áudio, preenchendo-o com zeros. Isso é crucial para garantir que não haja sons indesejados (lixo de memória) no início da aplicação do efeito.

```cpp
float apply_echo(Echo* e, float x) {
    float delayed = e->buffer[e->index];    // Get delayed sample
    float y = x + delayed * e->decay;       // Apply decay to delayed sample

    e->buffer[e->index] = y;                // Store new sample in buffer
    e->index = (e->index + 1) % e->size;    // Increment index circularly

    return y;
}

```

**`float delayed = e->buffer[e->index];`**: Lê uma amostra do *buffer* na posição atual do índice (`e->index`). Esta amostra é o som que foi armazenado `delay_samples` amostras atrás no tempo — ou seja, é o **som atrasado**.

**`float y = x + delayed * e->decay;`**: Calcula a nova amostra de saída (`y`). Ela é a **soma** da amostra de entrada atual (`x`) com a amostra atrasada (`delayed`), que por sua vez é multiplicada pelo fator de decaimento (`e->decay`) para reduzir seu volume.

**`e->buffer[e->index] = y;`**: **Armazena a nova amostra de saída (`y`) de volta no buffer**, na mesma posição de onde a amostra atrasada foi lida. Isso é o que cria as repetições contínuas (feedback). O som recém-criado será lido novamente no futuro para gerar o próximo eco.

### 5. Reverb 📳:

O código implementa uma forma simplificada de **reverberação** (reverb), baseada em uma única linha de atraso com realimentação (*feedback*).

```cpp
// Initialize the reverb effect
void reverb_init(Reverb* r, float delay_ms, float feedback, float mix, float sample_rate) {
    int delay_samples = (int)(sample_rate * delay_ms / 1000.0f);                // Convert delay time to samples
    if (delay_samples > MAX_DELAY_SAMPLES) delay_samples = MAX_DELAY_SAMPLES;   // Clamp to max size
    r->size = delay_samples;                                                    // Set size
    r->index = 0;                                                               // Reset index
    r->feedback = feedback;                                                     // Set feedback amount           
    r->mix = mix;                                                               // Set mix amount             
    memset(r->buffer, 0, sizeof(r->buffer));                                    // Clear buffer
}
```

A função `reverb_init` prepara o efeito, de maneira quase idêntica à do eco, mas com a adição de um novo parâmetro: `mix`.

`float mix`: O nível de mistura. Controla a proporção entre o som original e o som com efeito. Um valor de 0 significa nenhum reverb, e 1 significa apenas o som do reverb.

```cpp
float apply_reverb(Reverb* r, float x) {
    float delayed = r->buffer[r->index];                // Get delayed sample
    float y = x * (1.0f - r->mix) + delayed * r->mix;   // Mix input with delayed sample

    r->buffer[r->index] = x + delayed * r->feedback;    // Store new sample in buffer
    r->index = (r->index + 1) % r->size;                // Increment index circularly

    return y;
}
```

**`float y = x * (1.0f - r->mix) + delayed * r->mix;`**: Esta é a linha de **mistura da saída**. Ela calcula o que o ouvinte irá escutar.

* `x * (1.0f - r->mix)`: Pega a amostra de entrada (`x`) e a multiplica pela proporção de som sem efeito (`1 - mix`).
* `delayed * r->mix`: Pega a amostra atrasada (`delayed`) e a multiplica pela proporção de som com efeito (`mix`).
* O resultado `y` é a soma ponderada do som original e do som reverberado. Este controle de `mix` é típico de efeitos de reverb e permite ajustar o quão "distante" o som parece estar.

**`r->buffer[r->index] = x + delayed * r->feedback;`**: Esta é a linha da **realimentação (feedback)**. Ela calcula o que será armazenado de volta no buffer para criar as próximas reflexões. No eco, a *saída final* era armazenada de volta no buffer. Aqui, o que é armazenado é a **soma da entrada atual (`x`) com a reflexão anterior atenuada (`delayed * r->feedback`)**.

### 6. Pitch Shifter ◀️▶️:

Este código implementa um efeito de **pitch shifter**, que altera a "altura" (a frequência fundamental) de um som.

```cpp
// Initialize the pitch shifter
// pitch_factor: e.g. 0.7 for ~7 semitones down
void pitchshifter_init(PitchShifter* ps, float pitch_factor, float sample_rate) {
    memset(ps->buffer, 0, sizeof(ps->buffer));  // Clear buffer
    ps->write_index = 0;                        // Reset write index
    ps->read_index = 0.0f;                      // Reset read index
    ps->pitch_factor = pitch_factor;            // Set pitch factor
    ps->size = MAX_DELAY_SAMPLES;               // Set size to max delay samples
}
```

`float pitch_factor`: O **fator de afinação**. Este é o número mais importante.

* Um `pitch_factor` de **1.0** não altera o som.
* Um `pitch_factor` **< 1.0** (ex: 0.5) **diminui** a afinação (som mais grave).
* Um `pitch_factor` **> 1.0** (ex: 2.0) **aumenta** a afinação (som mais agudo).

`int size`: O tamanho do *buffer* de memória que será usado para armazenar o áudio temporariamente.

**`ps->read_index = 0.0f;`** : Inicializa o **ponteiro de leitura** . Note que ele é um `float`, pois ele se moverá em incrementos fracionários para alcançar a mudança de velocidade.

**`ps->write_index = 0;`** : Inicializa o **ponteiro de escrita** . Este é um `int` porque ele sempre se move uma amostra por vez.

```cpp
// Linear interpolation helper
static float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

// Process one sample with pitch shifting down
float apply_pitchshifter(PitchShifter* ps, float input) {
    ps->buffer[ps->write_index] = input;                    // Store input sample in buffer 

    // Calculate read index
    float output = 0.0f;                                    // Read sample at slower rate for pitch down
    int idx1 = (int)ps->read_index;                         // Get integer part of read index
    int idx2 = (idx1 + 1) % ps->size;                       // Get next index circularly
    float frac = ps->read_index - idx1;                     // Fractional part for interpolation

    // Linear interpolate between two samples
    output = lerp(ps->buffer[idx1], ps->buffer[idx2], frac);

    // Increment write index
    ps->write_index = (ps->write_index + 1) % ps->size;

    // Increment read index slower for pitch down
    ps->read_index += ps->pitch_factor;
    if (ps->read_index >= ps->size) {
        ps->read_index -= ps->size;
    }

    return output;
}
```

**Leitura e Interpolação**: Como o ponteiro de leitura (`read_index`) é um `float`, ele raramente cairá exatamente em uma posição inteira do *buffer*. Tentar ler o valor de `buffer[10.5]` não é possível. A solução é a **interpolação linear**.

* `int idx1 = (int)ps->read_index;`: Pega a parte inteira do ponteiro de leitura (ex: 10).
* `int idx2 = (idx1 + 1) % ps->size;`: Pega o próximo índice no *buffer* (ex: 11).
* `float frac = ps->read_index - idx1;`: Pega a parte fracionária (ex: 0.5).
* `output = lerp(ps->buffer[idx1], ps->buffer[idx2], frac);`: A função `lerp` (Interpolação Linear) calcula um valor intermediário. Se `frac` é 0.5, ela retorna a média exata entre o valor em `idx1` e `idx2`. Isso "adivinha" qual seria o valor da onda sonora no ponto fracionário, resultando em um som muito mais suave do que simplesmente arredondar o índice.

`ps->write_index = (ps->write_index + 1) % ps->size;`: O ponteiro de escrita **sempre avança de 1 em 1**, seguindo o ritmo normal do áudio.

`ps->read_index += ps->pitch_factor;`: O ponteiro de leitura **avança na velocidade do `pitch_factor`**.

* Se `pitch_factor` é 0.5, o ponteiro de leitura se move na metade da velocidade do de escrita. Ele está "lendo o passado" mais devagar, o que estica a onda sonora, resultando em um som mais grave.
* Se `pitch_factor` é 2.0, ele lê duas vezes mais rápido, "pulando" amostras. Isso comprime a onda sonora, resultando em um som mais agudo.

### 7. Filtros customizados:

Todos os filtros customizados não introduzem um novo tipo de processamento de áudio, mas sim criam uma **cadeia de efeitos** (*effect chain*), combinando vários dos filtros e efeitos que vimos anteriormente para alcançar um resultado sonoro final, como por exemplo o efeito do **Darth Vader**.

```cpp
float apply_darthvader(DarthVader* dv, float x) {

    x = apply_pitchshifter(&dv->ps, x);                 // Apply pitch shifter
    x = apply_distortion(x, dv->distortion_threshold);  // Apply distortion
    x = apply_equalizer(&dv->eq, x);                    // then equalizer
    x = apply_reverb(&dv->reverb, x);                   // reverb
    x = apply_volume_gain(x, dv->volume_gain);          // volume gain last

    return x;
}
```

---
## Clonagem de Voz por IA

A clonegem de voz foi feita usando uma aplicação *Open-Source* chamada **Retrieval-based-Voice-Conversion-WebUI**. É possível seguir o passo a passo disponibilizado no Readme, em português, no seguinte [link](https://github.com/RVC-Project/Retrieval-based-Voice-Conversion-WebUI/blob/main/docs/pt/README.pt.md) do GitHub. Para **inferência**, é necessário uma máquina com **VRAM de 6Gb na placa de vídeo**. Já para o **treinamento** de um novo filtro de voz, por exemplo o famoso filtro do professor Zambon, é requisito mínimo **12Gb de VRAM na placa de vídeo**.

# Work in Progress
## Saída de som amplificada pelo dispositivo
Pretendemos adicionar um módulo amplificador PAM8403 ligado a saída do DAC para permitir uma interface adicional que permite o uso de alto-falantes sem pré-aplificação. Desse modo, planejamos conectar em sequencia ao PAM um speaker simples para uma saída de áudio adicional.

O problema desse novo apêndice é a realimenação do áudio do speaker de saída volta no microfone de entrada, podendo gerar uma microfonia que deve ser tratada de alguma forma a ser investida posteriormente (via código e/ou de modo infraestrutural).

## Circuito completo com alimentação via bateria e interface
A ideia é que o dispositivo completo seja realmente portátil! Assim, pretendemos finalizar o circuito com uma parte de alimentação via baterias 9V para possibilitar o uso do aparelho sem que você estaja ancorado por um cabo de alimentação.

Adicionalmente, pretendemos adiconar uma interface com o usuário para permitir a seleção e visualização dos filtros atráves de botões e um display LCD.

## Novos filtros
Existe uma infinidade de possibilidades para construção de novos filtros, seja isso concatenando filtros existentes ou programando alguns novos. Por enquanto externamos 6 filtros diferentes, mas a ideia é criar ainda mais!

# Referências
(coisas que estão no doc "Referências" no ClickUp)
