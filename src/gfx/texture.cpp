#include "texture.h"

#include <iostream>
#include <string>

#define STBI_ONLY_PNG
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

namespace gfx {

GLuint TextureBase::freeIndex = 0;

TextureBase::TextureBase() {
    glGenTextures(1, &id);
    index = freeIndex++;
}

TextureBase::~TextureBase() { glDeleteTextures(1, &id); }

GLuint TextureBase::getIndex() const { return index; }

Texture::Texture(const char *fileName) {
    stbi_set_flip_vertically_on_load(true);
    loadTexture(fileName);
}

Texture::Texture(util::fs::path filePath) {
    stbi_set_flip_vertically_on_load(true);
    loadTexture(filePath.string().c_str());
}

void Texture::loadTexture(const char *fileName) {
    int width, height, nChannels;
    stbi_uc *data = stbi_load(fileName, &width, &height, &nChannels, 0);
    if (data == nullptr) {
        std::cerr << fileName << " not found!" << std::endl;
        return;
    }
    int colorFormat = (nChannels == 4) ? GL_RGBA : GL_RGB;
    glActiveTexture(GL_TEXTURE0 + index);
    glBindTexture(GL_TEXTURE_2D, id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, colorFormat, width, height, 0, colorFormat, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
    stbi_image_free(data);
}

ShadowMapTexture::ShadowMapTexture(unsigned int size) {
    shadowSize = size;
    GLfloat borderColor[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    glGenFramebuffers(1, &depthMapFBO);

    glActiveTexture(GL_TEXTURE0 + getIndex());
    glBindTexture(GL_TEXTURE_2D, id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, size, size, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);

    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, id, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

ShadowMapTexture::~ShadowMapTexture() { glDeleteFramebuffers(1, &depthMapFBO); }

unsigned int ShadowMapTexture::getShadowSize() const { return shadowSize; }

void ShadowMapTexture::bindFrameBuffer() const { glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO); }

void ShadowMapTexture::unbindFrameBuffer() const { glBindFramebuffer(GL_FRAMEBUFFER, 0); }

CubeTexture::CubeTexture(const std::array<const char *, 6> &fileName) {
    stbi_set_flip_vertically_on_load(false);
    loadTexture(fileName);
}

CubeTexture::CubeTexture(const std::array<util::fs::path, 6> &fileName) {
    stbi_set_flip_vertically_on_load(false);
    std::array<std::string, 6> stringPath;
    std::array<const char *, 6> cstringPath;
    for (int i = 0; i < 6; ++i) {
        stringPath[i] = fileName[i].string();
        cstringPath[i] = stringPath[i].c_str();
    }
    loadTexture(cstringPath);
}

void CubeTexture::loadTexture(const std::array<const char *, 6> &fileName) {
    int width = 0, height = 0, nChannels = 0;
    stbi_uc *data = nullptr;
    glActiveTexture(GL_TEXTURE0 + index);
    glBindTexture(GL_TEXTURE_CUBE_MAP, id);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    for (int i = 0; i < 6; ++i) {
        data = stbi_load(fileName[i], &width, &height, &nChannels, 0);
        if (data == nullptr) {
            std::cerr << fileName[i] << " not found!" << std::endl;
            return;
        }
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
        stbi_image_free(data);
    }
}
}  // namespace gfx
