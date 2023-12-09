'''
Projeto: Programa de visão computacional para robô seguidor de linha
Autor: João Gabriel Almeida Ferreira
Última atualização: 01 de maio de 2023
'''

import numpy as np      # Importa a biblioteca Numpy
import matplotlib as mp # Importa a biblioteca MatPlotLib
import cv2 as cv        # Importa a biblioteca OpenCV
import requests as rq   # Importa a biblioteca Requests

# Definição de cores
Vermelho = (0, 0, 255)
Verde    = (0, 255, 0)
Azul     = (255, 0, 0)
Amarelo  = (0, 255, 255)
Preto    = (0, 0, 0)
Branco   = (255, 255, 255)

# Área mínima para um contorno ser considerado
ÁreaMínima = 500

# Definição da fonte usada
Fonte = cv.FONT_HERSHEY_SIMPLEX

# Quanto menor o limiar, mais escuro deve ser o preto
LimiarBinarizacao = 125    

def VisãoComputacional(Imagem): 

    '''TRATAMENTO DA IMAGEM'''
    Altura, Largura  = Imagem.shape[:2]                                                       # Obtém as dimensões da imagem
    CentroVertical = int(Altura / 2)
    CentroHorizontal = int(Largura / 2)

    PretoBranco = cv.cvtColor(Imagem, cv.COLOR_BGR2GRAY)                                      # Converte a imagem de RGB  para escala de cinza
    #PretoBranco = cv.GaussianBlur(PretoBranco, (21, 21), 0)                                  # Aplica desfoque gaussiano à imagem

    FrameBinarizado = cv.threshold(PretoBranco, LimiarBinarizacao, 255, cv.THRESH_BINARY)[1] # Realiza a binarização da imagem
                                                                                             
    #FrameBinarizado = cv.dilate(FrameBinarizado, None, iterations = 2)                      # Tratamento de imagem
    #FrameBinarizado = cv.erode(FrameBinarizado, None, iterations = 2)                       # Tratamento de imagem
    
    Kernel = np.ones((3,3), np.uint8)                                                        # Tratamento de imagem      
    FrameBinarizado = cv.morphologyEx(FrameBinarizado, cv.MORPH_OPEN, Kernel)                # Tratamento de imagem

    #FrameBinarizado = cv.bitwise_not(FrameBinarizado)                                       # Inverte a cor dos pixeis, para linhas pretas em fundos brancos
    
    # Obtém os contornos da região branca do frame binarizado
    Contornos, Hierarquia = cv.findContours(FrameBinarizado.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    # Parâmetros: Imagem, Contornos, Quais contornos imprimir ("-1" são todos), Cor, Espessura
    cv.drawContours(Imagem, Contornos,-1, Azul, 2)                                  

    '''APLICAÇÃO DE VISÃO COMPUTACIONAL'''
    if len(Contornos) > 0:

        # Considera apenas um contorno por vez
        Contorno = Contornos[0]

        if cv.contourArea(Contorno) > ÁreaMínima:

            # Obtém o retângulo (pontos máximos) que circunscreve o contorno 
            (X_Total, Y_Total, W_Total, H_Total) = cv.boundingRect(Contorno)

            # Obtém o paralelogramo (pontos mínimos) que circunscreve o contorno
            Paralelogramo = cv.minAreaRect(Contorno)
            Caixa = cv.boxPoints(Paralelogramo)
            Caixa = np.int0(Caixa)

            #Obtém informações sobre o paralelogramo, que podem ser úteis para previsão
            (X_Min, Y_Mmin), (W_Min, H_Min), Ângulo = Paralelogramo
            Ângulo = int(Ângulo)

            # Desenhando as duas formas na imagem
            cv.rectangle(Imagem, (X_Total, Y_Total), (X_Total + W_Total, Y_Total + H_Total), Azul, 2)
            cv.drawContours(Imagem, [Caixa], 0, Amarelo, 2)

           # O trecho de código a seguir não é necessário, pois não trabalharemos com aproximações matemáticas de linhas,
           # mas com a distância (erro) entre pontos centrais do contorno e pontos de referência arbitrados (set-point).
           
            '''
            # Obtém uma aproximação matemática do contorno, dividindo-o em subcontornos calculados
            Precisão = 0.01
            Coeficiente = Precisão * cv.arcLength(Contorno, False)
            Aproximação = cv.approxPolyDP(Contorno, Coeficiente, False)

            # Divide a aproximação em várias sub-regiões
            Subcontornos = []
            for i in range(len(Aproximação)):
                
                Subcontorno = Aproximação[i:i+2]
                if len(Subcontorno) == 2: Subcontornos.append(Subcontorno)
            
            # Desenha os sub-contornos na imagem
            cv.drawContours(Imagem, sub_contours, -1, Verde, 3)   
            '''

            '''CÁLCULO DOS OUTPUTS'''

            # Número de pontos de referência a serem determinados
            NúmeroLinhas = 5

            # Lista que armazenará os valores em pixel das alturas de cada ponto de referência
            Alturas = []

            # Distância mínima entre pontos na horizontal para que sejam considerados diferentes
            Tolerância_X = 60
            
            # Distância máxima na vertical para um ponto ser considerado interseção
            Tolerância_Y = 10

            # Lista com os valores das distâncias de X médio de cada altura em relação à linha de referência
            Distâncias = []

            # Cálculo dos valores em pixel das alturas de cada ponto de referência
            for Linha in range(NúmeroLinhas + 1):
                
                Divisões = int(Altura * (Linha / NúmeroLinhas)) 
                Alturas.append(Divisões)
                cv.line(Imagem, (0, int(Divisões)), (Largura, int(Divisões)), Vermelho, 2)      

            # Percorre cada valor das alturas
            for Altura in Alturas:
                
                Imprimir = False # Variável Booleana que só será verdadeira caso seja detectada uma interseção
                X_Totais = []    # Lista que armazenará os valores de X encontrados em cada iteração das alturas
                X_Anterior = 0   # Variável auxiliar que receberá o valor de X anterior, para corrigir as médias

                # Percorre cada um dos pontos do contorno
                for i in range(len(Contorno)):  

                    # Obtém as coordenadas X e Y de cada ponto do contorno
                    Coordenadas = tuple(Contorno[i][0])
                    X_Atual = Coordenadas[0]
                    Y_Atual = Coordenadas[1]

                    # Checa se há interseção entre o contorno e as alturas de referência
                    if abs(Y_Atual - Altura) < Tolerância_Y:
                        
                        # Desenha um círculo amarelo no ponto de interseção
                        #cv.circle(Imagem, (X_Atual, Altura), 5, Amarelo, -2)

                        # Se os pontos horizontais forem adjacentes, ignora-os
                        if abs(X_Atual - X_Anterior) < Tolerância_X:
                            X_Atual = X_Anterior
                            continue
                       
                        # Desenha um círculo verde no ponto médio de interseção
                        cv.circle(Imagem, (X_Atual, Altura), 5, Verde, -2)

                        # Armazena o valor médio de X e atualiza a variável auxiliar
                        X_Totais.append(X_Atual)
                        X_Anterior = X_Atual

                        # Se forem detectados mais de um ponto de interseção (o que é necessário), calcula a média entre eles
                        # Apenas um ponto de interseção pode induzir a erros indesejados
                        if len(X_Totais) > 1:
                            X_Médio = int(np.mean(X_Totais))
                            Imprimir = True

                # Se forem detectados pontos de interseção,
                if Imprimir ==  True:
                    Distâncias.append(CentroHorizontal - X_Médio)
                    cv.circle(Imagem, (X_Médio, Altura), 5, Vermelho, -2)
                
                # Se não forem detectadas interseções, considera um erro
                else:
                    Distâncias.append("Erro")

            # Percorre cada valor de altura, imprimindo seu respectivo valor de distância
            for Índice in range(len(Alturas)):
                
                # Imprime as distâncias referentes a cada altura
                print("Altura [", Índice,"]: ", Distâncias[Índice], sep = '')
                
                # Em caso de detecção das interseções, imprime na imagem o valor de seu respectivo erro 
                if Distâncias[Índice] != "Erro":
                    cv.putText(Imagem, str(Distâncias[Índice]),(CentroHorizontal - Distâncias[Índice], Alturas[Índice]-10), Fonte, 0.8, Vermelho, 1, cv.LINE_AA)

            # Imprime o ângulo de inclinação do paralelogramo na imagem
            cv.putText(Imagem, str(Ângulo),(50, Altura - 50), Fonte, 0.8, Branco, 1, cv.LINE_AA)

    # Imprime a linha central de referência na imagem
    cv.line(Imagem, (CentroHorizontal, 0), (CentroHorizontal, Altura), Vermelho, 2)

    # Exibe a imagem com todas as formas inseridas
    cv.imshow('Analise de Rota', Imagem)

while(True):

    # "Frame" recebe o quadro atual em forma de um array NumPy
    Frame = cv.imread("LinhaBranca.png")                            

    if Frame is not None:                                           # Se frames foram detectados...
        VisãoComputacional(Frame)                                   # Envia o frame para ser tratado

        if cv.waitKey(22) & 0xFF == ord('g'):                       # Se "G" for pressionada...
            break                                                   # Encerra a exibição

    else:                                                           # Se frames não forem detectados...
        print("Nenhum frame foi detectado!")                        # Imprime uma mensagem de erro na tela
        break
