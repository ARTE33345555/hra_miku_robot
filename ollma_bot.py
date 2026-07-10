from telegram import Update
from telegram.ext import (
    Application,
    CommandHandler,
    MessageHandler,
    ContextTypes,
    filters,
)
from ollama import chat

BOT_TOKEN = "YOUR_TELEGRAM_BOT_TOKEN"

MODEL = "llama3"


async def start(update: Update, context: ContextTypes.DEFAULT_TYPE):
    await update.message.reply_text(
        "Hello! I am Miku AI powered by Ollama."
    )


async def message(update: Update, context: ContextTypes.DEFAULT_TYPE):
    user_text = update.message.text

    await update.message.chat.send_action("typing")

    try:
        response = chat(
            model=MODEL,
            messages=[
                {
                    "role": "system",
                    "content": (
                        "You are Hatsune Miku, a friendly AI assistant."
                    ),
                },
                {
                    "role": "user",
                    "content": user_text,
                },
            ],
        )

        answer = response["message"]["content"]

    except Exception as e:
        answer = f"Error: {e}"

    await update.message.reply_text(answer)


def main():
    app = Application.builder().token(BOT_TOKEN).build()

    app.add_handler(CommandHandler("start", start))
    app.add_handler(
        MessageHandler(filters.TEXT & ~filters.COMMAND, message)
    )

    print("Bot started!")

    app.run_polling()


if __name__ == "__main__":
    main()
